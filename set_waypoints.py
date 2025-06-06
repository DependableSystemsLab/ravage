"""
This script is used to set waypoints for a vehicle using MAVLink.
It connects to the vehicle, loads waypoints from a file, uploads them to the vehicle, arms the vehicle, initiates takeoff, and starts the mission.
"""
from pymavlink import mavutil
import time
from ravage import load_config

# Connect to the vehicle
master = mavutil.mavlink_connection('udp:127.0.0.1:14550') 
master.wait_heartbeat()
print("Heartbeat received, connection established!")

def load_waypoints(filename):
    """
    Reads waypoints from a file and parses them into a list of dictionaries.

    Args:
        filename (str): The path to the waypoint file.

    Returns:
        list: A list of dictionaries, each representing a waypoint.
    """
    waypoints = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        
        # Check the file header
        if not lines[0].startswith('QGC WPL 110'):
            raise ValueError("Invalid waypoint file format")
        
        # Parse waypoints
        for i, line in enumerate(lines[1:]):
            values = line.strip().split()
            if len(values) != 12:
                raise ValueError(f"Invalid waypoint format at line {i+2}: {line}")
            
            waypoint = {
                'seq': int(values[0]),
                'current': int(values[1]),
                'frame': int(values[2]),
                'command': int(values[3]),
                'param1': float(values[4]),
                'param2': float(values[5]),
                'param3': float(values[6]),
                'param4': float(values[7]),
                'x_lat': float(values[8]),
                'y_long': float(values[9]),
                'z_alt': float(values[10]),
                'autocontinue': int(values[11])
            }
            waypoints.append(waypoint)
    return waypoints

def upload_waypoints(waypoints):
    """
    Uploads a list of waypoints to the vehicle.

    Args:
        waypoints (list): A list of dictionaries, each representing a waypoint.
    """
    print("Starting waypoint upload...")

    # Step 1: Clear existing waypoints
    master.waypoint_clear_all_send()
    print("Cleared existing waypoints.")

    # Step 2: Send total waypoint count
    time.sleep(1)  # Small delay for safety
    master.waypoint_count_send(len(waypoints))
    print(f"Sending waypoint count: {len(waypoints)}")

    # Step 3: Send each waypoint
    for wp in waypoints:
        master.mav.mission_item_send(
            master.target_system,  # Target system
            master.target_component,  # Target component
            wp['seq'],  # Sequence number
            wp['frame'],  # Coordinate frame
            wp['command'],  # MAV_CMD
            wp['current'],  # Current waypoint
            wp['autocontinue'],  # Autocontinue
            wp['param1'], wp['param2'], wp['param3'], wp['param4'],  # Params
            wp['x_lat'],  # Latitude
            wp['y_long'],  # Longitude
            wp['z_alt']  # Altitude
        )
        print(f"Uploaded waypoint {wp['seq']} to ({wp['x_lat']}, {wp['y_long']}, {wp['z_alt']})")
        time.sleep(0.1)  # Short delay between waypoints

    # Step 4: Wait for MISSION_ACK
    ack_received = False
    while not ack_received:
        ack_msg = master.recv_match(type='MISSION_ACK', blocking=True)
        if ack_msg:
            print(f"Mission ACK received: {ack_msg}")
            ack_received = True

    print("Waypoint upload complete!")

def arm_and_takeoff(altitude):
    """
    Arms the vehicle, switches to GUIDED mode, and initiates takeoff to the specified altitude.

    Args:
        altitude (float): The target altitude for takeoff in meters.
    """
    print("Setting mode to GUIDED...")
    master.set_mode('GUIDED')
    print("Mode set to GUIDED.")

    # Wait for mode confirmation
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == master.mode_mapping()['GUIDED']:
            print("Mode confirmed as GUIDED.")
            break
        time.sleep(1)

    # Wait for home position (valid GPS lock)
    print("Waiting for home position to be set (valid GPS fix)...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and msg.lat != 0 and msg.lon != 0:
            print(f"Home position set: Latitude={msg.lat / 1e7}, Longitude={msg.lon / 1e7}")
            break
        else:
            print("Waiting for valid GPS fix...")
        time.sleep(1)

    # Arm the motors
    print("Arming motors...")
    master.arducopter_arm()
    print("Motors armed.")

    # # Send takeoff command
    # print(f"Sending takeoff command to {altitude} meters...")
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Command
    #     0, 0, 0, 0, 0, 0, 0, altitude  # Altitude
    # )

    # # Wait for acknowledgment of takeoff command
    # while True:
    #     msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
    #         if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    #             print("Takeoff command accepted.")
    #             break
    #         else:
    #             print("Takeoff command failed. Retrying...")
    #             raise Exception("NAV_TAKEOFF command failed. Check GPS and home position.")
    #     time.sleep(1)

    # Monitor altitude to confirm takeoff
    # print("Takeoff initiated. Monitoring altitude...")
    # while True:
    #     msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    #     altitude_current = msg.relative_alt / 1000.0  # Altitude in meters
    #     print(f"Current altitude: {altitude_current:.2f} meters")
    #     if altitude_current >= altitude * 0.95:  # 95% of target altitude
    #         print("Target altitude reached!")
    #         break
    #     time.sleep(1)

def start_mission():
    """
    Starts the mission execution from the first waypoint.
    """
    print("Starting mission...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,  # Command to start the mission
        0,  # Confirmation
        0,  # First waypoint to start from
        0, 0, 0, 0, 0, 0  # Unused parameters
    )

    # Wait for confirmation
    while True:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_MISSION_START:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Mission started successfully!")
                break
            else:
                print("Mission start command failed. Retrying...")
                raise Exception("MAV_CMD_MISSION_START command failed.")
        time.sleep(1)

def monitor_position():
    """
    Monitors the vehicle's position (latitude, longitude, altitude) and logs when it starts moving towards a waypoint.
    """
    print("Monitoring vehicle position...")

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7  # Convert from int32 to degrees
            lon = msg.lon / 1e7  # Convert from int32 to degrees
            alt = msg.relative_alt / 1000.0  # Convert from mm to meters
            print(f"Current Position - Latitude: {lat:.6f}, Longitude: {lon:.6f}, Altitude: {alt:.2f}m")

            # Check if the vehicle is moving (simple check for significant latitude/longitude changes)
            if lat != 0 and lon != 0:
                print(f"Vehicle is moving towards a waypoint. Position - Lat: {lat}, Lon: {lon}")
                break
        
        time.sleep(1)  # Adjust the sleep interval as needed

def main():
    """
    Main function to execute the waypoint setting process.
    """
    try:
        config_path = "config/autopilot_config.yaml"
        config = load_config(config_path)
        wp_file = config.get('WAYPOINT_FILE', 'square_copter.txt')
        # Load waypoints from file
        waypoint_file = 'waypoints/' + wp_file
        waypoints = load_waypoints(waypoint_file)
        print(f"Loaded waypoints: {waypoints}")

        # Upload waypoints to the vehicle
        upload_waypoints(waypoints)

        # Arm and takeoff
        arm_and_takeoff(altitude=100)  # Takeoff to 100 meters

        # Start the mission
        start_mission()
        #TODO: Buggy, need to fix
        #monitor_position()

        print("Mission execution in progress...")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

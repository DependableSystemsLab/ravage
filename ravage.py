"""
This script is used to inject attacks on ArduPilot or PX4 software by manipulating various parameters.
It parses command line arguments to determine the attack type, intensity, and software platform.
Based on these inputs, it performs the attack, logs the status, and resets the parameters to their original values.
"""
import argparse
import threading
import time
import yaml
import csv
import os
import math
import numpy as np
from pymavlink import mavutil
from datetime import datetime

# Global Variables
Parachute_on = 0
GPS_status = 1
Accel_status = 1
Gyro_status = 1
PreArm_error = 0
Baro_status = 1
EKF_lane_switch = 0
GPS_glitch = 0
failsafe = 0
crash_on_ground = 0
vibration_compensation = 0
disarming_motors = 0
gps_failsafe_error = 0
timestamp = ""
csv_file_created = False

# New global variables for mission tracking
current_position = {"lat": 0, "lon": 0, "alt": 0}
waypoints = []
current_waypoint_idx = 0
prev_waypoint_idx = 0
max_path_deviation = 0  # Maximum deviation from path in meters
cumulative_deviation = 0  # Cumulative deviation over time
deviation_samples = 0  # Number of samples for averaging
deviation_threshold = 5  # Meters - configurable threshold for successful attack

def log_attack_status(software, sensor, deviation, crash_on_ground, attack_result):
    """
    Logs the status of the attack to a CSV file.

    Args:
        software (str): The software platform (ArduPilot or PX4).
        sensor (str): The sensor being attacked.
        deviation (float): The maximum deviation from path in meters.
        crash_on_ground (int): Flag indicating if a crash on the ground occurred.
        attack_result (str): The result of the attack ("attack_running", "attack_success", or "attack_fail").
    """
    
    global timestamp, csv_file_created
    # Create log directory if it doesn't exist
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Get the current timestamp
    if not csv_file_created:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file_created = True
    
    # Define the log filename
    log_filename = f"{log_dir}/{timestamp}_{software.lower()}_{sensor.lower()}.csv"
    
    # Check if the file exists; if not, write the header
    file_exists = os.path.exists(log_filename)

    impact = int(GPS_glitch or failsafe or disarming_motors or gps_failsafe_error)

    with open(log_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            # Write header if the file is new
            writer.writerow(["Timestamp", "Deviation_meters", "Impact", "Crash", "Attack_result"])
        
        # Write the current status
        writer.writerow([timestamp, deviation, impact, crash_on_ground, attack_result])

def handle_status(msg):
    """
    Handles status updates and sets corresponding flags based on the received MAVLink messages.

    Args:
        msg (MAVLink_message): The received MAVLink message.
    """
    global Parachute_on, GPS_status, Accel_status, Gyro_status, PreArm_error
    global Baro_status, EKF_lane_switch, GPS_glitch, failsafe, crash_on_ground
    global vibration_compensation, disarming_motors, gps_failsafe_error

    # Print the full status text for debugging
    print(f"[Status Message] {msg.text}")

    # Parachute Detection
    if "Parachute: Released" in msg.text:
        Parachute_on = 1
        print("[CRITICAL] Parachute has been deployed!")

    # GPS and Navigation Checks
    elif "NavEKF" in msg.text and "lane switch" in msg.text:
        GPS_status = 0
        print("[Warning] NavEKF lane switch detected - GPS status compromised")

    elif "GPS Glitch" in msg.text:
        GPS_glitch = 1
        print("[Alert] GPS Glitch detected")

    elif "Failsafe enabled: no global position" in msg.text:
        gps_failsafe_error = 1
        print("[Critical] GPS Failsafe - No global position")

    # Sensor-Specific Checks
    # Gyroscope
    elif "Vibration compensation ON" in msg.text:
        Gyro_status = 0
        vibration_compensation = 1
        print("[Warning] High vibration detected - Gyro compensation activated")

    # Accelerometer
    elif "EKF primary changed" in msg.text:
        Accel_status = 0
        print("[Alert] EKF primary changed - Potential accelerometer issue")

    # Barometer
    elif "PreArm: Waiting for Nav Checks" in msg.text:
        Baro_status = 0
        print("[Warning] Barometer navigation checks pending")

    # PreArm and System Checks
    elif "PreArm: Check" in msg.text:
        PreArm_error = 1
        print("[Alert] PreArm check failed")

    # System State Checks
    elif "lane switch" in msg.text:
        EKF_lane_switch = 1
        print("[Warning] EKF lane switch detected")

    elif ("Failsafe" in msg.text) or ("failsafe" in msg.text) or ("Disarmed by landing" in msg.text):
        failsafe = 1
        print("[Critical] Failsafe condition triggered")

    elif "Hit ground at" in msg.text:
        crash_on_ground = 1
        print("[Critical] Vehicle hit ground")

    elif "disarm" in msg.text:
        disarming_motors = 1
        print("[Alert] Motors disarmed")

    # Magnetometer Checks
    if "Compass" in msg.text:
        if "inconsistent" in msg.text.lower():
            GPS_glitch = 1
            print("[MAG Attack] Compass data inconsistent")
        elif "failure" in msg.text.lower():
            GPS_glitch = 1
            print("[MAG Attack] Compass failure detected")
        elif "interference" in msg.text.lower():
            GPS_glitch = 1
            print("[MAG Attack] Magnetic interference detected")
        elif "calibration" in msg.text.lower() and "failed" in msg.text.lower():
            PreArm_error = 1
            print("[MAG Attack] Compass calibration failed")

    # Optical Flow Checks
    if "OptFlow" in msg.text or "Optical Flow" in msg.text:
        if "invalid" in msg.text.lower():
            GPS_glitch = 1
            print("[OF Attack] Optical Flow data invalid")
        elif "no data" in msg.text.lower():
            GPS_glitch = 1
            print("[OF Attack] No Optical Flow data")
        elif "quality" in msg.text.lower() and "low" in msg.text.lower():
            PreArm_error = 1
            print("[OF Attack] Low Optical Flow quality")
        elif "sensor" in msg.text.lower() and "error" in msg.text.lower():
            GPS_glitch = 1
            print("[OF Attack] Optical Flow sensor error")

    # Extended Kalman Filter (EKF) Checks
    if "EKF" in msg.text:
        # Magnetometer EKF Checks
        if "mag" in msg.text.lower():
            if "variance" in msg.text.lower():
                EKF_lane_switch = 1
                print("[MAG Attack] EKF Magnetometer variance issue")
            elif "innovation" in msg.text.lower():
                EKF_lane_switch = 1
                print("[MAG Attack] EKF Magnetometer innovation limit exceeded")
        
        # Optical Flow EKF Checks
        if "optical flow" in msg.text.lower():
            if "variance" in msg.text.lower():
                EKF_lane_switch = 1
                print("[OF Attack] EKF Optical Flow variance issue")
            elif "innovation" in msg.text.lower():
                EKF_lane_switch = 1
                print("[OF Attack] EKF Optical Flow innovation limit exceeded")

    # Accelerometer Checks
    if "Accel" in msg.text or "Accelerometer" in msg.text:
        if "inconsistent" in msg.text.lower():
            Accel_status = 0
            PreArm_error = 1
            print("[Accel Attack] Accelerometer data inconsistent")
        elif "failure" in msg.text.lower():
            Accel_status = 0
            GPS_glitch = 1
            print("[Accel Attack] Accelerometer failure detected")
        elif "calibration" in msg.text.lower() and "failed" in msg.text.lower():
            Accel_status = 0
            PreArm_error = 1
            print("[Accel Attack] Accelerometer calibration failed")
        elif "error" in msg.text.lower():
            Accel_status = 0
            GPS_glitch = 1
            print("[Accel Attack] Accelerometer error detected")
        elif "bias" in msg.text.lower():
            Accel_status = 0
            EKF_lane_switch = 1
            print("[Accel Attack] Accelerometer bias detected")

def calculate_distance_to_path(lat, lon, wp1_lat, wp1_lon, wp2_lat, wp2_lon):
    """
    Calculate the perpendicular distance from a point to a line defined by two waypoints
    using the cross-track error formula.
    
    All coordinates are in degrees.
    Returns distance in meters.
    """
    # Convert to radians
    lat = math.radians(lat)
    lon = math.radians(lon)
    wp1_lat = math.radians(wp1_lat)
    wp1_lon = math.radians(wp1_lon)
    wp2_lat = math.radians(wp2_lat)
    wp2_lon = math.radians(wp2_lon)
    
    # Earth radius in meters
    R = 6371000
    
    # Calculate bearing from wp1 to wp2
    y = math.sin(wp2_lon - wp1_lon) * math.cos(wp2_lat)
    x = math.cos(wp1_lat) * math.sin(wp2_lat) - math.sin(wp1_lat) * math.cos(wp2_lat) * math.cos(wp2_lon - wp1_lon)
    bearing_wp1_to_wp2 = math.atan2(y, x)
    
    # Calculate bearing from wp1 to current position
    y = math.sin(lon - wp1_lon) * math.cos(lat)
    x = math.cos(wp1_lat) * math.sin(lat) - math.sin(wp1_lat) * math.cos(lat) * math.cos(lon - wp1_lon)
    bearing_wp1_to_pos = math.atan2(y, x)
    
    # Calculate angular distance from wp1 to current position
    d_wp1_to_pos = math.acos(math.sin(wp1_lat) * math.sin(lat) + 
                            math.cos(wp1_lat) * math.cos(lat) * math.cos(lon - wp1_lon))
    
    # Calculate cross-track distance (XTD)
    xtd = math.asin(math.sin(d_wp1_to_pos) * math.sin(bearing_wp1_to_pos - bearing_wp1_to_wp2))
    
    # Convert to meters
    distance = abs(xtd * R)
    
    return distance

def update_position_tracking(msg):
    """
    Update the global position tracking variables based on received position data.
    
    Args:
        msg (MAVLink_message): The received MAVLink position message.
    """
    global current_position, waypoints, current_waypoint_idx, max_path_deviation, cumulative_deviation, deviation_samples, prev_waypoint_idx
    
    # Update current position
    if msg.get_type() == 'GLOBAL_POSITION_INT':
        # Convert from int (1e7) to degrees
        current_position["lat"] = msg.lat / 1e7
        current_position["lon"] = msg.lon / 1e7
        current_position["alt"] = msg.alt / 1000.0  # Convert from mm to m
        
        # Calculate deviation from mission path if we have waypoints
        if len(waypoints) > 1 and current_waypoint_idx > 0:
            # Check if current_waypoint_idx is valid before trying to access it
            if current_waypoint_idx < len(waypoints):  
                # Get current path segment (previous waypoint to current target)
                wp1 = waypoints[current_waypoint_idx - 1]
                wp2 = waypoints[current_waypoint_idx]
                
                # Calculate distance to the path
                distance = calculate_distance_to_path(
                    current_position["lat"], current_position["lon"],
                    wp1["lat"], wp1["lon"],
                    wp2["lat"], wp2["lon"]
                )
                
                # Update maximum deviation
                max_path_deviation = max(max_path_deviation, distance)
                
                # Update cumulative stats for average calculation
                cumulative_deviation += distance
                deviation_samples += 1
                
                # Log significant deviations
                if distance > deviation_threshold:
                    print(f"[Path Deviation] {distance:.2f}m from planned route")
            else:
                print(f"[Warning] Current waypoint index {current_waypoint_idx} is out of range (max: {len(waypoints)-1})")
    
    # Update current waypoint tracking
    elif msg.get_type() == 'MISSION_CURRENT':
        prev_waypoint_idx = current_waypoint_idx
        current_waypoint_idx = msg.seq
        if prev_waypoint_idx != current_waypoint_idx:
            print(f"[Navigation] Now targeting waypoint {current_waypoint_idx}")

    # Store mission waypoints when received
    elif msg.get_type() == 'MISSION_ITEM' or msg.get_type() == 'MISSION_ITEM_INT':
        # Convert from int (1e7) to degrees if needed
        lat = msg.x / 1e7 if msg.get_type() == 'MISSION_ITEM_INT' else msg.x
        lon = msg.y / 1e7 if msg.get_type() == 'MISSION_ITEM_INT' else msg.y
        
        # Add or update waypoint
        if msg.seq < len(waypoints):
            waypoints[msg.seq] = {"lat": lat, "lon": lon, "alt": msg.z}
        else:
            waypoints.append({"lat": lat, "lon": lon, "alt": msg.z})
            
        # Debug
        print(f"[Waypoint] Added/updated waypoint {msg.seq} at {lat:.6f}, {lon:.6f}, {msg.z:.2f}m")

def request_mission_data(master):
    """
    Request the full mission from the vehicle.
    This is important to have the waypoints for deviation calculations.
    
    Args:
        master (MAVLink_connection): The MAVLink connection.
    """
    # Get mission count first
    master.mav.mission_request_list_send(
        master.target_system,
        master.target_component
    )
    
    msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if not msg:
        print("[Warning] Couldn't get mission count")
        return
    
    print(f"[Mission] Vehicle has {msg.count} mission items")
    
    # Now request each mission item
    for i in range(msg.count):
        master.mav.mission_request_send(
            master.target_system,
            master.target_component,
            i
        )
        time.sleep(0.1)  # Small delay to not overload the autopilot
    
    print("[Mission] Mission waypoints requested")

def set_param(master, name, value):
    """
    Sets a parameter on the vehicle.

    Args:
        master (MAVLink_connection): The MAVLink connection object.
        name (str): The name of the parameter to set.
        value (float): The value to set the parameter to.
    """
    param_id = name.encode('utf-8')

    try:
        # Send parameter set command
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_id,
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        print(f"[Info] Sent parameter set command for {name} with value {value}.")

        # Request parameter value for verification
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param_id,
            -1
        )
        print(f"[Info] Sent parameter request command for {name}.")

        # Wait for the response
        response = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if response:
            response_dict = response.to_dict()
            print(f"[Success] Parameter {name} set to {response_dict['param_value']}.")
        else:
            print(f"[Error] No response for parameter {name}.")

    except Exception as e:
        print(f"[Exception] Error setting parameter {name}: {str(e)}")

def get_param(master, param_name, retries=3):
    """
    Gets the current value of a parameter.

    Args:
        master (MAVLink_connection): The MAVLink connection object.
        param_name (str): The name of the parameter to get.
        retries (int): How many times the function should retry to retreieve the parameter value

    Returns:
        float: The current value of the parameter, or None if the parameter is not received.
    """
    for _ in range(retries):
        master.mav.param_request_read_send(
            master.target_system, 
            master.target_component, 
            param_name.encode('utf-8'),
            -1
        )
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if message:
            return message.param_value
        time.sleep(1)  # Wait before retrying
    print(f"Warning: Parameter {param_name} not received after {retries} attempts.")
    return None

def reset_all_params(master, params, original_values):
    """
    Resets all parameters to their original values and verifies them.

    Args:
        master (MAVLink_connection): The MAVLink connection object.
        params (list): The list of parameters to reset.
        original_values (dict): A dictionary of original parameter values.
    """
    print("Resetting all parameters to their original values...")
    
    for param, value in original_values.items():
        set_param(master, param, value)  # Reset each parameter to its original value

    print("All parameters have been reset. Verifying...")

    # Verify that parameters have been restored correctly
    for param, original_value in original_values.items():
        current_value = get_param(master, param)
        if current_value == original_value:
            print(f"[Verified] {param} correctly reset to {current_value}.")
        else:
            print(f"[Warning] {param} expected {original_value} but found {current_value}.")

def load_config(config_path):
    """
    Loads the configuration from the specified YAML file.

    Args:
        config_path (str): The path to the YAML configuration file.

    Returns:
        dict: The configuration dictionary loaded from the YAML file.
    """
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def is_attack_success():
    """
    Determines if an attack is successful based on various failure conditions
    or significant path deviation.
    """
    # Original failure criteria
    failure_conditions = [
        GPS_glitch, failsafe, crash_on_ground, disarming_motors, gps_failsafe_error,
        Parachute_on, not GPS_status, not Accel_status, not Gyro_status, PreArm_error,
        not Baro_status, EKF_lane_switch, vibration_compensation
    ]
    
    # Add path deviation as a success criterion
    if max_path_deviation > deviation_threshold:
        print(f"[Attack Success] Maximum path deviation of {max_path_deviation:.2f}m exceeds threshold of {deviation_threshold}m")
        return True
    
    # Check original criteria
    return any(failure_conditions)

def get_avg_deviation():
    """
    Calculates the average deviation from the path.
    
    Returns:
        float: Average deviation in meters, or 0 if no samples.
    """
    if deviation_samples > 0:
        return cumulative_deviation / deviation_samples
    return 0

# Function to perform attack
def perform_attack(master, software, attack_type, intensity, duration, config, sensor_config):
    """
    Performs the specified attack by manipulating parameters.

    Args:
        master (MAVLink_connection): The MAVLink connection object.
        software (str): The software platform (ArduPilot or PX4).
        attack_type (str): The type of attack to perform.
        intensity (float): The intensity of the attack.
        duration (float): The duration of the attack.
        config (dict): The configuration dictionary for parameters.
        sensor_config (dict): The configuration dictionary for sensors.
    """
    global max_path_deviation, cumulative_deviation, deviation_samples
    
    # Request mission data to get waypoints for path deviation calculations
    request_mission_data(master)
    
    # Reset deviation metrics before starting attack
    max_path_deviation = 0
    cumulative_deviation = 0
    deviation_samples = 0
    
    # Fetch attack parameters based on software and attack type
    if software == "ArduPilot":
        params = config[attack_type]["ArduPilot"]
    elif software == "PX4":
        params = config[attack_type]["PX4"]
    else:
        print("Unsupported software!")
        return

    # Fetch sensor-specific attack parameters
    sensor = sensor_config.get(attack_type, {})
    time_duration = duration
    bias_range = sensor.get('bias_range', [-10, 10])
    bias_pattern = sensor.get('bias_pattern', "constant")
    attack_result = "attack_running"
    
    # Fetch the deviation threshold from config or use default
    global deviation_threshold
    deviation_threshold = sensor_config.get('deviation_threshold', 5)  # Default 5 meters
    
    # Store the original values of the parameters
    original_values = {}
    for param in params:
        value = get_param(master, param)
        if value is not None:
            original_values[param] = value
        else:
            print(f"Skipping parameter {param} as it was not received.")
    
    # Define the attack parameters based on bias pattern
    min_bias, max_bias = bias_range
    current_bias = intensity  # Initial bias value from intensity

    if bias_pattern == "increasing":
        increment = 0.5  # Define the increment for increasing pattern
    elif bias_pattern == "decreasing":
        increment = -0.5  # Define the decrement for decreasing pattern
    else:
        increment = 0  # No change if constant

    start_time = time.time()
    last_log_time = start_time
    
    while time.time() - start_time < time_duration:
        for param in original_values.keys():
            print(f"Injecting attack: {attack_type} on {param} with value {current_bias}")
            set_param(master, param, current_bias)
        
        # Adjust the bias based on the pattern
        if bias_pattern == "increasing":
            current_bias = min(current_bias + increment, max_bias)
        elif bias_pattern == "decreasing":
            current_bias = max(current_bias + increment, min_bias)
        elif bias_pattern == "constant":
            current_bias = intensity  # Keep the value constant

        # Log attack status periodically (every 5 seconds)
        current_time = time.time()
        if current_time - last_log_time >= 5:
            if attack_result == "attack_running":
                attack_result = "attack_success" if is_attack_success() else attack_result
                log_attack_status(software, attack_type, max_path_deviation, crash_on_ground, attack_result)
                last_log_time = current_time

        # Sleep to simulate the attack duration between parameter adjustments
        time.sleep(1)

    print(f"Attack completed. Total attack duration: {time_duration} seconds.")
    print(f"Maximum path deviation: {max_path_deviation:.2f}m")
    print(f"Average path deviation: {get_avg_deviation():.2f}m")
    
    # After the attack, reset all parameters to their original values
    reset_all_params(master, params, original_values)
    
    # Final evaluation of attack success
    if attack_result == "attack_running":
        attack_result = "attack_success" if is_attack_success() else "attack_fail"
        log_attack_status(software, attack_type, max_path_deviation, crash_on_ground, attack_result)
    
    # Hard reset if using ArduPilot to prevent home address issues
    if software == "ArduPilot":
        set_param(master, 'FORMAT_VERSION', 0)

# Function to run the message reader in a separate thread
def read_loop(master, stop_event):
    """
    Runs the message reader in a separate thread to continuously read MAVLink messages.

    Args:
        master (MAVLink_connection): The MAVLink connection object.
        stop_event (threading.Event): The event to signal the thread to stop.
    """
    while not stop_event.is_set():
        # Grabbing mavlink message
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue

        # Handle the message based on its type
        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            handle_status(msg)
        elif msg_type in ["GLOBAL_POSITION_INT", "MISSION_CURRENT", "MISSION_ITEM", "MISSION_ITEM_INT"]:
            update_position_tracking(msg)

# Parse command line arguments
def parse_arguments():
    """
    Parses command line arguments.

    Returns:
        argparse.Namespace: The parsed command line arguments.
    """
    parser = argparse.ArgumentParser(description="Fuzzing script to inject attacks on ArduPilot or PX4")
    parser.add_argument('-i', '--intensity', type=float, required=True, help="Intensity of attack")
    parser.add_argument('-t', '--time_duration', type=float, required=False, help="Time duration of attack")
    parser.add_argument('-s', '--software', choices=['ArduPilot', 'PX4'], required=True, help="Software platform (ArduPilot or PX4)")
    parser.add_argument('-a', '--attack', required=True, help="Type of attack (e.g., GPS, GYRO)")
    parser.add_argument('-d', '--deviation', type=float, required=False, help="Path deviation threshold in meters to consider attack successful (default: 5)")
    return parser.parse_args()

# Main function to run the script
def main():
    """
    Main function to run the script. Parses arguments, loads configuration, connects to MAVLink, and performs the attack.
    """
    args = parse_arguments()
    time_duration = 0
    
    # Set deviation threshold if provided
    global deviation_threshold
    if args.deviation is not None:
        deviation_threshold = args.deviation
    
    # Load config from YAML files
    param_config = load_config('config/param_config.yaml')  # Path to your parameter config file
    sensor_config = load_config('config/attack_profile.yaml')  # Path to your sensor config file

    # Check if time_duration was passed; if not, use the default from the sensor configuration
    if args.time_duration is None:
        time_duration = sensor_config[args.attack].get('time', 60)  # Default to 60 seconds if not specified
    else:
        time_duration = args.time_duration
    
    # Connect to MAVLink
    print(f"Connecting to {args.software}...")
    connection_string = 'udp:127.0.0.1:14550'
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print(f"Heartbeat received from {args.software}!")

    # Create a stop event to signal the message reader to stop
    stop_event = threading.Event()

    # Start the message reading thread
    message_thread = threading.Thread(target=read_loop, args=(master, stop_event))
    message_thread.start()

    # Give time to collect initial waypoints and set up tracking
    print("Waiting for mission data...")
    time.sleep(10)  

    # Run attack in a separate thread to allow continuous fuzzing during the attack duration
    attack_thread = threading.Thread(target=perform_attack, args=(master, args.software, args.attack, args.intensity, time_duration, param_config, sensor_config))
    attack_thread.start()

    # Wait for the attack to complete
    attack_thread.join()

    # Signal the message thread to stop
    stop_event.set()

    # Wait for the message thread to finish
    message_thread.join()

if __name__ == "__main__":
    main()
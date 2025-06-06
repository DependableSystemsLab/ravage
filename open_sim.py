"""
This script is used to start a simulation for a specified vehicle type using either ArduPilot or PX4 software.
It parses command line arguments to determine the vehicle type, software path, software version, and whether to wipe the EEPROM.
Based on these inputs, it constructs and executes the appropriate command to start the simulation.
"""

from subprocess import Popen
import sys, getopt

def main(argv):
    """
    Main function to parse command line arguments and execute the appropriate simulation command.

    Args:
        argv (list): List of command line arguments.
    """
    wipe_eeprom = "off"
    software_path = ""
    software_version = ""

    # (Start) Parse command line arguments (i.e., input and output file)
    try:
        opts, args = getopt.getopt(argv, "v:wp:s:", ["vehicle_type=", "sw_path=","sw_version"])
    except getopt.GetoptError:
        print("[open_sim.py] Error with parsing cmd line arguments")
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-v", "--vehicle_type"):
            vehicle_target = str(arg)
            print("[open_sim.py] Probing type of vehicles: %s" % vehicle_target)
        if opt == '-w':
            wipe_eeprom = "on"
            print("[open_sim.py] Wipe EEPROM and reload configuration parameters")
        if opt in ("-p", "--sw_path"):
            software_path = str(arg)
        if opt in ("-s", "--sw_version"):
            software_version = str(arg)

    # (End) Parse command line arguments (i.e., input and output file)

    SOFTWARE_VER = software_version
    SOFTWARE_HOME = software_path
    if software_path is None:
        raise Exception("SOFTWARE_HOME environment variable is not set!")

    if wipe_eeprom == "off" and SOFTWARE_VER == "ArduPilot":
        c = SOFTWARE_HOME + 'Tools/autotest/sim_vehicle.py -v ' + vehicle_target + ' --console --map'
    elif wipe_eeprom == "on" and SOFTWARE_VER == "ArduPilot":
        c = SOFTWARE_HOME + 'Tools/autotest/sim_vehicle.py -v ' + vehicle_target + ' --console --map -w'
    else:
        # c = 'cd ' + SOFTWARE_HOME + ' && make px4_sitl ' + vehicle_target
        c = 'cd ' + SOFTWARE_HOME + ' && make px4_sitl jmavsim'

    handle = Popen(c, shell=True)
    print(c)

if __name__ == "__main__":
    main(sys.argv[1:])
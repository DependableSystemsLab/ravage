"""
Mission script to start a simulation for a specified vehicle type using either ArduPilot or PX4 software.
It parses command line arguments to determine the software version and whether to wipe the EEPROM.
Based on these inputs, it loads the configuration, starts the simulator, and sets waypoints.
"""

from pymavlink import mavutil
import time
from subprocess import *
import sys, os, getopt
import yaml
from ravage import load_config

# Connection type over UDP
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

RAVAGE_HOME = ""
SOFTWARE_HOME = ""
wipe_eeprom = "off"
start_sw_version = ""

def set_waypoints():
    """
    Function to set waypoints for the mission.
    It opens a new terminal and runs the set_waypoints.py script.
    """
    c = 'gnome-terminal -- bash -c "python ' + RAVAGE_HOME + 'set_waypoints.py; exec bash" &'

    handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
    print(c)

# -------------------------------------------------------------------------------------------------------------
def open_simulator(vehicle_type, allow_wipe_eeprom, start_sw_version):
    """
	Function to open the simulator for the specified vehicle type and software version.

	Args:
		vehicle_type (str): The type of vehicle to simulate.
		allow_wipe_eeprom (bool): Whether to allow wiping the EEPROM.
		start_sw_version (str): The software version to use (ArduPilot or PX4).
    """
    global RAVAGE_HOME
    global SOFTWARE_HOME
    global wipe_eeprom

    c = 'gnome-terminal -- bash -c "python ' + RAVAGE_HOME + 'open_sim.py -v ' + str(vehicle_type) + ' -p ' + SOFTWARE_HOME + ' -s ' + str(start_sw_version)

    if wipe_eeprom == "on" and allow_wipe_eeprom is True:
        c += ' -w'

    c += '; exec bash" &'
    
    handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
    print(c)

    # Waiting for initializing SITL
    time.sleep(55)
	
# -------------------------------------------------------------------------------------------------------------
def main(argv):
	"""
    Main function to parse command line arguments, load configuration, and start the simulation.

    Args:
        argv (list): List of command line arguments.
    """
	global RAVAGE_HOME
	global SOFTWARE_HOME
	global wipe_eeprom
	global start_sw_version

	# (Start) Parse command line arguments (i.e., input and output file)
	try:
		opts, args = getopt.getopt(argv, "hws:", ["sw_version="])
		
	except getopt.GetoptError:
		print("Incorrect command line arguments")
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-h':
			print("ravage.py help") #TODO
		if opt == '-w':
			wipe_eeprom = "on"
			print("Wipe EEPROM and reload configuration parameters")	
		if opt in ("-s", "--sw_version"):
			start_sw_version = str(arg)
			print("Using {} software".format(start_sw_version))

	# (End) Parse command line arguments (i.e., input and output file)
	#----------------------------------------------------------------------
	if(start_sw_version == "ArduPilot"):
		#default vehicle
		vehicle_type = "ArduCopter" #Copter default
		
		config_path = "config/autopilot_config.yaml"
		config = load_config(config_path)

		RAVAGE_HOME = config.get('RAVAGE_HOME')
		SOFTWARE_HOME = config.get('ARDUPILOT_HOME')

		#Set vehicle type (currently only possible to choose one): currently priority encoded
		if(config.get('ROVER') == 'Yes'):
			vehicle_type = "APMrover2"
		elif (config.get('COPTER') == 'Yes'):
			vehicle_type = "ArduCopter"
		elif (config.get('SUB') == 'Yes'):
			vehicle_type = "ArduSub"

		print("*{}* will be tested".format(vehicle_type))
		#Opens SIM
		open_simulator(vehicle_type, True, start_sw_version)
		#Upload waypoints 
		set_waypoints()

	elif(start_sw_version == "PX4"):
		#default vehicle
		vehicle_type = "gazebo_iris_opt_flow" #Copter default
		
		config_path = "config/autopilot_config.yaml"
		config = load_config(config_path)

		RAVAGE_HOME = config.get('RAVAGE_HOME')
		SOFTWARE_HOME = config.get('PX4_HOME')

		#Set vehicle type (currently only possible to choose one): currently priority encoded
		if(config.get('ROVER') == 'Yes'):
			vehicle_type = "gazebo_rover"
		elif (config.get('COPTER') == 'Yes'):
			vehicle_type = "gazebo_iris_opt_flow"
		elif (config.get('SUB') == 'Yes'):
			vehicle_type = "gazebo_uuv_hippocampus"

		print("*{}* will be tested".format(vehicle_type))
		#Opens SIM
		open_simulator(vehicle_type, True, start_sw_version)
		#TODO: Setting waypoints feature for PX4

if __name__ == "__main__":
   main(sys.argv[1:])

# RAVAGE: Robotic Autonomous Vehicles' Attack Generation Engine

**RAVAGE** is an attack injection tool that enables realistic physical attacks (e.g., GPS spoofing, gyroscope and accelerometer tampering) on robotic autonomous vehicles (RAVs) such as drones and rovers. RAVAGE is designed to be:

- **Extensible:** Easily integrates with multiple autopilot software platforms (e.g., ArduPilot, PX4).
- **Configurable:** Allows fine-tuning of attack parameters (e.g., duration, intensity) without modifying autopilot code.
- **Non-invasive:** Supports seamless attack injection in both virtual and real RAV environments.

---

## Key Features
- Accurately emulates physical attacks via software.
- Seamless integration with both virtual and real RAV systems.
- Compatible with multiple RAV platforms and vehicle types.

---

## Supported Tools
RAVAGE has been tested with both drones and rovers using the following autopilot systems:

- [**ArduPilot**](https://ardupilot.org/dev/docs/building-setup-linux.html)  
- [**PX4**](https://docs.px4.io/main/en/dev_setup/dev_env.html)

> PX4 supports Gazebo and JSBSim for simulation. We also recommend installing [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

---

## Running RAVAGE
There are two ways to run RAVAGE:

1. **Install the required tools manually** (recommended for experienced users).
2. **Use our pre-configured Ubuntu VM** for easier setup (recommended for quick deployment).

---

## Prerequisites
Ensure you have the following dependencies:
- **Python 3.X**
- **pymavlink**
- **yaml**

---

## Setup Instructions

### 1. Configure Autopilot Settings
Edit the configuration file `./config/autopilot_config.yaml` to specify your environment:

```yaml
RAVAGE_HOME: "/path/to/ravage/home/"
ARDUPILOT_HOME: "/path/to/ardupilot/home/"
PX4_HOME: "/path/to/px4/home/"
WAYPOINT_FILE: "square_copter.txt"
ROVER: "No"
COPTER: "Yes"
```

**Parameters:**
- `RAVAGE_HOME`: Path to the RAVAGE project home.
- `ARDUPILOT_HOME`: Path to the ArduPilot installation.
- `PX4_HOME`: Path to the PX4 installation.
- `WAYPOINT_FILE`: File containing mission waypoints.
- `ROVER`: Set to `Yes` for rover simulation, otherwise `No`.
- `COPTER`: Set to `Yes` for copter simulation, otherwise `No`.
---

### 2. Configure Attack Profiles
Define the attack parameters in `./config/attack_profile.yaml` to specify attack types, intensity, and duration.

---

### 3. Launch Autopilot and Simulator
Run the following command to start the autopilot and set waypoints:

```bash
python mission.py -s ArduPilot
```

**Options:**
- `-s`: Specify the autopilot system (`ArduPilot` | `PX4`)

---

### 4. Launch and Configure Sensor Attacks
Use the following command to inject attacks:

```bash
python ravage.py -s <autopilot> -a <attack_type> -i <intensity> -d <duration>
```

**Options:**
- `-s`, `--autopilot`: Autopilot software (`ArduPilot` | `PX4`)
- `-a`, `--attack_type`: Attack type (`GPS` | `GYRO` | `MAG` | `OPTICAL_FLOW` | `ACCELEROMETER`)
- `-i`, `--intensity`: Attack intensity (float value)
- `-d`, `--duration`: Attack duration (integer value)

---

## Example Workflow
1. Edit the configuration files (`autopilot_config.yaml` and `attack_profile.yaml`).
2. Start the simulation and set waypoints:

```bash
python mission.py -s ArduPilot
```

3. Inject a GPS spoofing attack with intensity `3`:

```bash
python ravage.py -s ArduPilot -a GPS -i 3
```

This example initializes the simulation, sets mission waypoints, and performs a GPS spoofing attack during the mission.

## Citation
If you find our work useful in your research, please consider citing:

```
@inproceedings{dash2025ravage,
      title={RAVAGE: Robotic Autonomous Vehicles' Attack Generation Engine}, 
      author={Dash, Pritam and Pattabiraman, Karthik},
      booktitle = {The 55th Annual IEEE/IFIP International Conference on Dependable Systems and Networks (DSN)},
      year={2025}
}
```

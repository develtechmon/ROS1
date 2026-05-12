# ROS1 Noetic + Ubuntu 20.04 + PX4 v1.13.3 + AirSim Setup Guide (Linux)

## Overview

This guide provides a complete setup for PX4 SITL simulation with Gazebo Classic, integrated with ROS1 Noetic, AirSim on Ubuntu Local Fossa.

**What you'll achieve:**
- PX4 Autopilot v1.13.3 running Ubuntu 20.04
- Gazebo Classic simulation with various vehicle types
- Installing matching AirSim version and PX4 version

**System Requirements:**
- Windows 11 with Ubuntu dual boot installed
- Ubuntu 20.04 in 
- 8GB RAM minimum (16GB recommended)
- 50GB free disk space


### Step 1: Install PX4 dependencies
```
# Install dependencies
sudo apt install -y \
    python3-pip \
    python3-jinja2 \
    python3-empy \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-dev \
    ninja-build \
    exiftool \
    astyle

# Install additional tools
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly
```

and Install AirSim Python Package
```
pip install --upgrade pip setuptools setuptools_scm wheel build
pip install msgpack==0.6.2 msgpack-python==0.5.6 msgpack-rpc-python==0.4.1
pip install --upgrade numpy
pip install airsim

or
pip install airsim --no-build-isolation
```

### Step 2: Clone PX4 Repository
```
cd ~

git clone https://github.com/PX4/PX4-Autopilot.git --recursive

cd PX4-Autopilot

git checkout v1.13.3

git submodule sync --recursive

git submodule update --init --recursive --force

bash ./Tools/setup/ubuntu.sh

make px4_sitl_default none_iris
```

First build takes 10-15 minutes

Expected output:
```
______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580
```

✅ PX4 is now waiting for AirSim to connect. But for now, we can quit it.

### Step 3: Clone AirSim repository
```
cd ~
git clone https://github.com/microsoft/AirSim.git
cd AirSim

# Use v1.6.0-linux (most stable for Linux)
git checkout v1.6.0-linux

# Update submodules
git submodule update --init --recursive
```

Why v1.6.0-linux?

    v1.8.1 has infinite loop bug in build.sh
    v1.6.0 is stable and tested


### Step 4: Build AirLib
```
cd ~/AirSim

# Run setup script
./setup.sh

# Build AirLib (this takes 5-10 minutes)
./build.sh
```

Expected output:
```
Setting up Eigen...
Setting up RpcLib...
Building AirLib...
Building MavLinkCom...
Building RPC library...
Build completed successfully!
```

If you see errors about infinite loops: You're on v1.8.1. Switch to v1.6.0-linux.

### Step 5: Verify Build
```
# Check if binaries exist
ls ~/AirSim/build_release/output/bin/

# Should show:
# MavLinkTest
# DroneServer
# HelloDrone
# DroneShell
```
✅ If these files exist, AirLib is compiled successfully!

### Step 5: Download AirSim simulator (Blocks)

Here, we'll be using `v1.8.1` Linux version. The link of this simulator located here:
```
https://github.com/microsoft/AirSim/releases/download/v1.8.1/Blocks.zip

Download here
wget https://github.com/microsoft/AirSim/releases/download/v1.8.1/Blocks.zip
unzip Blocks.zip
cd Blocks/LinuxEditor
./Blocks.sh -windowed
```

Please use  this `settings.json` if youre running `AirSim` using its own API not PX4 or Ardupiot. This is default `settings.json` content
```
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ViewMode": "FlyWithMe",
  "CameraDirector": {
    "FollowDistance": -3
  },
  "Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Armed",
      "AllowAPIAlways": true,
      "RC": {
        "RemoteControlID": 0,
        "AllowAPIWhenDisconnected": true
      }
    }
  }
}
```
Please use  this `settings.json` if youre running `PX4` for Software in the Loop
```
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "LocalHostIp": "127.0.0.1",
            "Sensors":{
                "Barometer":{
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                }
            },
            "Parameters": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1,
                "LPE_LAT": 47.641468,
                "LPE_LON": -122.140165
            }
        }
    }
}
```

Please use  this `settings.json` if youre running `PX4` for Hardware in the Loop
```
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": true,
            "SerialPort": "/dev/ttyACM0",
            "SerialBaudRate": 115200,
            "LockStep": false,
            "Sensors": {
                "Barometer": {
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                },
                "Gps": {
                    "SensorType": 3,
                    "Enabled": true
                },
                "Magnetometer": {
                    "SensorType": 4,
                    "Enabled": true
                },
                "Imu": {
                    "SensorType": 2,
                    "Enabled": true
                }
            },
            "Parameters": {
                "COM_ARM_WO_GPS": 1,
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_RC_IN_MODE": 4,
                "COM_OBL_ACT": 1,
                
                "EKF2_AID_MASK": 1,
                "EKF2_HGT_MODE": 0,
                "EKF2_GPS_CHECK": 0
            }
        }
    }
}

```


Let's launch if `PX4` and `AirSim` is working

Terminal 1 and waif for connection
```
./Blocks.sh -windowed
```

Terminal 2 perform make
```
cd PX4-Autopilot
make px4_sitl_default none_iris
```

Let's wait and you shoud see the AirSim says it's connected


## Step 6: Launch `mavros` for our testing.
To launch `mavros` use the following command in new terminal
```
roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14555" 
```

## Step 7: Disable RC failsafe.

In the PX4 console, run the additional parameters to disable RC failsafe in SITL. But in actual hardware, it's not recommended.
```
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param save
```
Explanation:

`COM_RCL_EXCEPT 4` = RC loss exception mode: allow all modes to continue without RC
`NAV_RCL_ACT 0` = When RC is lost, do nothing (you already set this, but double-check)

## Step 6 : Let's control the drone using Velocity mode

To control the drone using mavros velocity mode, we have to follow this step sequentially as follow

Terminal 1 - 1st we need perform the following
```
commander arm
commander takeoff
```

Terminal 2 - run the following command to check what is the current flight mode.
```
rostopic echo /mavros/state -n 1
```

Terminal 3 — Switch to OFFBOARD and check the mode once drone hovering ok.
```
# Switch to OFFBOARD
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

rostopic echo /mavros/state -n 1
```

Terminal 4 - Use this command during your test

Move Forward at 1 m/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Move Backward at 1 m/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: -1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Move Left at 1 m/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Move Right at 1 m/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: -1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Ascend at 0.5 m/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.5
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Descend at 0.5 m/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: -0.5
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
Rotate Left (Yaw) at 0.5 rad/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

Rotate Right (Yaw) at 0.5 rad/s:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.5"
```

Stop/Hover:
```
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Understanding Velocity Fields
```
linear:
  x: 1.0   # Forward (+) / Backward (-) in m/s
  y: 1.0   # Left (+) / Right (-) in m/s
  z: 1.0   # Up (+) / Down (-) in m/s
angular:
  x: 0.0   # Roll (rarely used)
  y: 0.0   # Pitch (rarely used)
  z: 0.5   # Yaw: rotate left (+) / right (-) in rad/s
```


# Getting Started 

This is userguide on how to install and setup `mavsdk` in your environment. The advantage of `mavsdk` it doesnt require `ROS` installation.

## Step 1: Install this package

Terminal 1

If you're using python3.9 and above, this package should be ok for you
```
pip3 install mavsdk
pip3 install aioconsole
```

if you're using python3.8, you should install a specific version
```
pip3 install "mavsdk<2.0.0"
pip3 install aioconsole
```

## Step 2: Select whether you're using Airsim or Gazebo

### AirSim 

Terminal 2
Run this command to launch `px4` SITL
```
make px4_sitl_default none_iris
```

Then next we're ready to run our `Airsim`.

If you're using AirSim can use this `settings.json` please use this value
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
		        "NAV_RCL_ACT": 0,
            		"NAV_DLL_ACT": 0,
            		"COM_OBL_RC_ACT": 1,
			"COM_RC_IN_MODE": 4,
			"COM_ARM_WO_GPS":1
            }
        }
    }
}

```
Then run the `airsim blocks`.
```
./Blocks.sh -ResX=640 -ReesY=480 -windowed
```

To test if `mavsdk` is working, please run the following command in new terminal

Terminal 3
Running this command will open a console for us to test the `mavsdk`
```
apython
```
Then in the console type the following command one by one
```
from mavsdk import System
drone = System()
await drone.connect(system_address="udp://:14550")

or this one to directly connect to PX4 outgoing port
await drone.connect(system_address="udp://127.0.0.1:14550")

await drone.action.arm()
await drone.action.takeoff()
```

It everything is working, you should able to see the drone is armed and takeoff. To land run  this command
```
await drone.action.takeoff()
```

If everything is OK, we can try to run our customer script included in this package.

### Gazebo SITL

Terminal 2
Run this command to launch `px4` SITL
```
make px4_sitl_default gazebo-classic_iris
```

or if you want to run using other method
```
HEADLESS=1 make px4_sitl gazebo-classic
```

And in other terminal
```
roslaunch gazebo_ros iris_world.launch
```

Then inside `px4 console` please set this parameter which you can find also in `gazebo hitl` guide
```
# ── FAILSAFE DISABLE ──────────────────────────────────────────────────────────
param set NAV_RCL_ACT 0          # No RC loss failsafe (no RC in HITL)
param set NAV_DLL_ACT 0          # No data link loss failsafe
param set COM_RC_IN_MODE 4       # No RC controller required
param set COM_OBL_RC_ACT 1       # Hover when offboard ends (v1.14 name)

# ── FLIP PARAMETERS ───────────────────────────────────────────────────────────
param set FD_FAIL_R 0            # Disable roll failure detector
param set FD_FAIL_P 0            # Disable pitch failure detector
param set MC_ROLLRATE_MAX 1800   # Allow 30 rad/s flip roll rate
param set COM_OF_LOSS_T 5        # Offboard timeout 5 seconds
param set CBRK_FLIGHTTERM 121212 # Disable flight termination system

# ── FASTER VERTICAL RECOVERY ──────────────────────────────────────────────────
param set MPC_Z_VEL_P_ACC 8.0   # 2x faster altitude recovery after flip
param set MPC_Z_VEL_MAX_UP 6.0  # Faster upward climb after flip
param set MPC_ACC_UP_MAX 10.0   # Higher upward acceleration

# ── EKF2 GPS RELAXATION ───────────────────────────────────────────────────────
param set EKF2_GPS_CHECK 0       # Disable GPS quality checks
param set EKF2_REQ_EPH 5.0      # Relaxed horizontal accuracy
param set EKF2_REQ_EPV 8.0      # Relaxed vertical accuracy
param set EKF2_REQ_SACC 5.0     # Relaxed speed accuracy
param set EKF2_REQ_NSATS 4      # Minimum 4 satellites
param set EKF2_REQ_HDRIFT 0.3   # Relaxed horizontal drift
param set EKF2_REQ_VDRIFT 0.5   # Relaxed vertical drift

# ── ARMING ────────────────────────────────────────────────────────────────────
param set COM_ARM_WO_GPS 1       # Allow arming without GPS

# ── MAVLINK ───────────────────────────────────────────────────────────────────
param set MAV_1_CONFIG 0         # Disable TELEM2 MAVLink (prevents Tx overflow)

# ── SAVE AND REBOOT ───────────────────────────────────────────────────────────
param save
reboot
```

To test if `mavsdk` is working, please run the following command in new terminal

Terminal 3
Running this command will open a console for us to test the `mavsdk`
```
apython
```
Then in the console type the following command one by one
```
from mavsdk import System
drone = System()
await drone.connect(system_address="udp://:14550")

or this one to directly connect to PX4 outgoing port
await drone.connect(system_address="udp://127.0.0.1:14550")

await drone.action.arm()
await drone.action.takeoff()
```

It everything is working, you should able to see the drone is armed and takeoff. To land run  this command
```
await drone.action.takeoff()
```

If everything is OK, we can try to run our customer script included in this package.


### Gazebo HITL

Please connect the `pixhawk` from windows to WSL. To connect, please refer to this userguide
```
https://github.com/develtechmon/ROS1/blob/master/UserGuide/Attach_Pixhawk_to_WSL_And_Connect_To_Gazebo.md
```

##  Connect Pixhawk with Gazebo

1. Step 1: Build Gazebo Classic Plugins (one time)
```
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic
```

2. Edit iri_hitl.sdf
```
vi ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl/iris_hitl.sdf
```
Find and set
```
<serialEnabled>1</serialEnabled>
<serialDevice>/dev/ttyACM0</serialDevice>
<baudRate>921600</baudRate>
<hil_mode>1</hil_mode>
```
save and exit.

3. Then follow this to launch
```
cd ~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash \
  $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world --verbose

# This launch bland iris drone world
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world --verbose

# This launch bayland grassfield drone world for better visualization
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris_env.world --verbose

```

You will see the serial device connected.

4. Set the `pixhawk` parameter .
   
You may need to set this parameter in `pixhawk` using `QGC` and the save and reboot and reconnect again.
```
# ── FAILSAFE DISABLE ──────────────────────────────────────────────────────────
param set NAV_RCL_ACT 0          # No RC loss failsafe (no RC in HITL)
param set NAV_DLL_ACT 0          # No data link loss failsafe
param set COM_RC_IN_MODE 4       # No RC controller required
param set COM_OBL_RC_ACT 1       # Hover when offboard ends (v1.14 name)

# ── FLIP PARAMETERS ───────────────────────────────────────────────────────────
param set FD_FAIL_R 0            # Disable roll failure detector
param set FD_FAIL_P 0            # Disable pitch failure detector
param set MC_ROLLRATE_MAX 1800   # Allow 30 rad/s flip roll rate
param set COM_OF_LOSS_T 5        # Offboard timeout 5 seconds
param set CBRK_FLIGHTTERM 121212 # Disable flight termination system

# ── FASTER VERTICAL RECOVERY ──────────────────────────────────────────────────
param set MPC_Z_VEL_P_ACC 8.0   # 2x faster altitude recovery after flip
param set MPC_Z_VEL_MAX_UP 6.0  # Faster upward climb after flip
param set MPC_ACC_UP_MAX 10.0   # Higher upward acceleration

# ── EKF2 GPS RELAXATION ───────────────────────────────────────────────────────
param set EKF2_GPS_CHECK 0       # Disable GPS quality checks
param set EKF2_REQ_EPH 5.0      # Relaxed horizontal accuracy
param set EKF2_REQ_EPV 8.0      # Relaxed vertical accuracy
param set EKF2_REQ_SACC 5.0     # Relaxed speed accuracy
param set EKF2_REQ_NSATS 4      # Minimum 4 satellites
param set EKF2_REQ_HDRIFT 0.3   # Relaxed horizontal drift
param set EKF2_REQ_VDRIFT 0.5   # Relaxed vertical drift

# ── ARMING ────────────────────────────────────────────────────────────────────
param set COM_ARM_WO_GPS 1       # Allow arming without GPS

# ── MAVLINK ───────────────────────────────────────────────────────────────────
param set MAV_1_CONFIG 0         # Disable TELEM2 MAVLink (prevents Tx overflow)

# ── SAVE AND REBOOT ───────────────────────────────────────────────────────────
param save
reboot
```

To test if `mavsdk` is working, please run the following command in new terminal

Terminal 3
Running this command will open a console for us to test the `mavsdk`
```
apython
```
Then in the console type the following command one by one
```
from mavsdk import System
drone = System()

await drone.connect(system_address="serial:///dev/ttyACM0:57600")

await drone.action.arm()
await drone.action.takeoff()
```

It everything is working, you should able to see the drone is armed and takeoff. To land run  this command
```
await drone.action.takeoff()
```

If everything is OK, we can try to run our customer script included in this package.


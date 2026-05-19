# ROS1 | Windows + WSL2 | Noetic | Ubuntu 20.04 | PX4 v1.14.3 | SITL Flip Mode

> **Purpose:** Complete reference for setting up and running autonomous drone flip and recovery maneuvers using PX4 v1.14.3 SITL, AirSim (Windows), and MAVROS (ROS Noetic on WSL2 Ubuntu 20.04).
>
> **Research Context:** PhD research on impact-resilient UAV systems. Flip maneuvers simulate impact disturbances for PPO reinforcement learning recovery testing.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Environment Setup](#2-environment-setup)
3. [Configuration Files](#3-configuration-files)
4. [PX4 Parameters](#4-px4-parameters)
5. [Startup Sequence](#5-startup-sequence)
6. [Flip Script](#6-flip-script)
7. [Troubleshooting](#7-troubleshooting)
8. [Key Lessons Learned](#8-key-lessons-learned)
9. [Next Steps: HITL](#9-next-steps-hitl)

---

## 1. System Overview

### Architecture

The system runs across two environments:
- **Windows** hosts AirSim for physics simulation and 3D visualization
- **WSL2 Ubuntu 20.04** runs PX4 SITL, MAVROS, and the Python flight control script
- Communication happens via TCP/UDP across the WSL2 virtual network bridge

```
┌─────────────────────────────────────────────────────────────────────┐
│ WINDOWS                                                             │
│  AirSim (Blocks.exe) — Physics + Visualization                     │
│  IP: 172.28.0.1 (WSL2 bridge)                                      │
└────────────────────────────┬────────────────────────────────────────┘
                             │ TCP 4560 (simulator)
                             │ UDP 14540/14580 (MAVLink onboard)
┌────────────────────────────▼────────────────────────────────────────┐
│ WSL2 UBUNTU 20.04                                                   │
│  ┌──────────────┐    UDP 14550/14555    ┌──────────────────────┐   │
│  │  MAVROS      │ ◄──────────────────► │  PX4 SITL v1.14.3    │   │
│  │  (ROS Noetic)│                       │  (none_iris)         │   │
│  └──────┬───────┘                       └──────────────────────┘   │
│         │ ROS topics                                                │
│  ┌──────▼───────┐                                                   │
│  │ drone_sitl_  │                                                   │
│  │ flip.py      │                                                   │
│  └──────────────┘                                                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Component Summary

| Component | Environment | Version | Role |
|-----------|-------------|---------|------|
| AirSim (Blocks.exe) | Windows | v1.8.1 | Physics simulation + 3D visualization |
| PX4 SITL | WSL2 Ubuntu 20.04 | v1.14.3 | Flight controller firmware in software |
| MAVROS | WSL2 Ubuntu 20.04 | ROS Noetic | MAVLink-to-ROS bridge for offboard control |
| drone_sitl_flip.py | WSL2 Ubuntu 20.04 | Python 3.8 | Flip + recovery control script |

### Communication Ports

| From | To | Port / Protocol | Purpose |
|------|----|-----------------|---------|
| AirSim (Win) | PX4 SITL (WSL2) | TCP 4560 | Simulator connection |
| PX4 SITL | AirSim | UDP 14540 / 14580 | MAVLink onboard (sensor data) |
| MAVROS | PX4 SITL | UDP 14550 / 14555 | MAVLink GCS (offboard commands) |

### WSL2 Network Bridge

WSL2 uses a virtual network adapter. The Windows host IP (`172.28.0.1`) is the bridge address.

```bash
# Check your WSL2 bridge IP (run inside WSL2)
cat /etc/resolv.conf | grep nameserver
# Output: nameserver 172.28.0.1  ← use this as PX4_SIM_HOST_ADDR
```

---

## 2. Environment Setup

### 2.1 Windows Requirements

- Windows 10/11 with WSL2 enabled
- AirSim Blocks binary (v1.4.0 Linux equivalent Windows build)
- GPU with Vulkan or DirectX 11+ support

### 2.2 WSL2 Ubuntu 20.04 Requirements

- Ubuntu 20.04 (Focal) on WSL2
- ROS Noetic (full desktop install)
- MAVROS + MAVROS extras
- PX4-Autopilot v1.14.3 cloned and built
- Python 3.8+

### 2.3 ROS Noetic Installation

```bash
# Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS Noetic
sudo apt update
sudo apt install ros-noetic-desktop-full

# Auto-source ROS
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.4 MAVROS Installation

```bash
# Binary install
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras

# Install required geographiclib datasets
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 2.5 PX4 v1.14.3 Setup

```bash
# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Checkout v1.14.3
git checkout v1.14.3
git submodule update --init --recursive

# Install dependencies
bash ./Tools/setup/ubuntu.sh --no-nuttx

# Build SITL
make px4_sitl_default none_iris
```

### 2.6 Verify PX4 Version

```bash
cd ~/PX4-Autopilot
git describe --tags
# Expected output: v1.14.3
```

---

## 3. Configuration Files

### 3.1 AirSim settings.json

**Location:** `C:\Users\<YourUser>\Documents\AirSim\settings.json`

> ⚠️ **IMPORTANT — PX4 v1.14.3 uses EKF2, NOT LPE.**
> Do NOT add `LPE_LAT` or `LPE_LON` to the Parameters block.
> These parameters do not exist in v1.14.3 and will crash AirSim with a runtime exception.

```json
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
      "ControlIp": "remote",
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LocalHostIp": "172.28.0.1",
      "Sensors": {
        "Barometer": {
          "SensorType": 1,
          "Enabled": true,
          "PressureFactorSigma": 0.0001825
        }
      },
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_OBL_RC_ACT": 1
      }
    }
  }
}
```

### 3.2 settings.json Parameters Explained

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `LocalHostIp` | `172.28.0.1` | Windows host IP (WSL2 bridge) — AirSim binds here |
| `UseTcp` | `true` | TCP for simulator connection (required for PX4 SITL) |
| `TcpPort` | `4560` | Port PX4 SITL listens on for simulator |
| `ControlIp` | `remote` | Accept control from remote (WSL2) IP |
| `LockStep` | `true` | Sync AirSim physics clock with PX4 — prevents drift |
| `ClockType` | `SteppableClock` | Required for LockStep to work correctly |
| `PressureFactorSigma` | `0.0001825` | Reduces barometer noise so PX4 EKF accepts GPS faster |
| `NAV_RCL_ACT: 0` | Disabled | RC loss failsafe action. We set 0=Disabled because there is no RC in SITL. See all values below. |
| `NAV_DLL_ACT: 0` | Disabled | Data link loss failsafe action. Set 0=Disabled so losing MAVROS connection does not crash the drone during testing. See all values below. |
| `COM_OBL_RC_ACT: 1` | Hold | Action when offboard connection is lost. Set 1=Hold so drone hovers if script stops. **Note: renamed from `COM_OBL_ACT` in PX4 v1.14 — using the old name crashes AirSim.** See all values below. |

**`NAV_RCL_ACT` — RC Loss Action:**

| Value | Behaviour |
|-------|-----------|
| `0` | **Disabled** — no action taken (use in SITL, no RC connected) |
| `1` | Loiter — hold position |
| `2` | Return to Launch (RTL) |
| `3` | Land immediately |
| `4` | Terminate flight |
| `5` | Lockdown (kill motors) |

**`NAV_DLL_ACT` — Data Link (GCS/MAVROS) Loss Action:**

| Value | Behaviour |
|-------|-----------|
| `0` | **Disabled** — no action taken (use in SITL) |
| `1` | Loiter — hold position |
| `2` | Return to Launch (RTL) |
| `3` | Land immediately |
| `4` | Terminate flight |
| `5` | Lockdown (kill motors) |

**`COM_OBL_RC_ACT` — Offboard Connection Loss Action** *(renamed from `COM_OBL_ACT` in v1.14)*:

| Value | Behaviour |
|-------|-----------|
| `0` | Position mode (if RC available) or Land |
| `1` | **Hold/Hover** — drone stays put (recommended for SITL testing) |
| `2` | Return to Launch (RTL) |
| `3` | Land immediately |
| `4` | Disarm |
| `5` | Terminate flight |

---

## 4. PX4 Parameters

### 4.1 Required Parameters for Flip

Run these in the PX4 console (`pxh>` prompt) **after** PX4 SITL starts and connects to AirSim:

```bash
# Allow arming without RC controller
param set COM_RC_IN_MODE 4

# Disable roll failure detector
# Value >360 completely disables it (allows full 360° flip)
param set FD_FAIL_R 0

# Disable pitch failure detector
param set FD_FAIL_P 0

# Allow 1800 deg/s max roll rate
# Required because 30 rad/s = ~1718 deg/s
param set MC_ROLLRATE_MAX 1800

# Offboard timeout — 5 seconds buffer during flip
param set COM_OF_LOSS_T 5

# Save to persistent storage (REQUIRED — resets on restart otherwise)
param save
```

### 4.2 Why Each Parameter Matters

| Parameter | Value | Why It Is Needed |
|-----------|-------|-----------------|
| `COM_RC_IN_MODE` | `4` | Default requires RC controller to arm. Setting 4 disables this so MAVROS can arm autonomously without a physical RC transmitter. |
| `FD_FAIL_R` | `400` | Roll failure detector threshold in degrees. Default is 60°. During a flip the drone rolls 360°. Without this PX4 triggers failsafe at 60° and kills motors — drone crashes. Setting >360 effectively disables it. |
| `FD_FAIL_P` | `400` | Same as FD_FAIL_R but for pitch axis. Set both together to allow full aerobatic maneuvers. |
| `MC_ROLLRATE_MAX` | `1800` | Maximum roll rate in deg/s. Our flip commands 30 rad/s ≈ 1718 deg/s. This must be higher than the commanded roll rate or PX4 clamps it. |
| `COM_OF_LOSS_T` | `5` | Offboard timeout in seconds. If no setpoint received for this long PX4 exits offboard mode. 5 seconds gives buffer during flip. |

**`COM_RC_IN_MODE` — RC Input Mode:**

| Value | Behaviour |
|-------|-----------|
| `0` | RC transmitter required — default |
| `1` | Joystick/RC allowed but not required |
| `2` | RC disabled during missions only |
| `3` | RC disabled in auto modes |
| `4` | **Stick input disabled** — use this for fully autonomous MAVROS control |

**`FD_FAIL_R` / `FD_FAIL_P` — Attitude Failure Detector Thresholds:**

| Value | Behaviour |
|-------|-----------|
| `0` | Disabled completely |
| `1–60` | Active — triggers failsafe if roll/pitch exceeds this angle for >0.3s |
| `60` | **Default** — triggers failsafe at 60° (too low for flips) |
| `400` | **Effectively disabled** — 400° > 360° so never triggers |

**`COM_OBL_RC_ACT` — Offboard Loss Action** *(v1.14 replacement for `COM_OBL_ACT`)*:

| Value | Behaviour |
|-------|-----------|
| `0` | Position mode (if RC available) or Land |
| `1` | **Hold/Hover** ← recommended for SITL testing |
| `2` | Return to Launch |
| `3` | Land immediately |
| `4` | Disarm |
| `5` | Terminate |

### 4.3 Verifying Parameters Were Saved

```bash
# Check specific parameters in PX4 console
param show FD_FAIL_R
param show FD_FAIL_P
param show MC_ROLLRATE_MAX
param show COM_RC_IN_MODE
param show COM_OF_LOSS_T

# Show all parameters changed from factory defaults
param show-for-airframe changed

# Via MAVROS from WSL2 terminal
rosrun mavros mavparam get FD_FAIL_R
rosrun mavros mavparam dump ~/px4_params_backup.txt
```

---

## 5. Startup Sequence

> ⚠️ **Always follow this exact order.** AirSim must start before PX4 SITL because PX4 immediately tries to connect to AirSim on TCP port 4560.

### Step 1: Start AirSim on Windows

Open Command Prompt or PowerShell on Windows:

```cmd
cd C:\Path\To\Blocks\WindowsNoEditor
.\Blocks.exe -ResX=640 -ResY=480 -windowed
```

Wait until the Blocks environment fully loads (blue/gray world visible with drone on the ground).

### Step 2: Start PX4 SITL in WSL2

```bash
# Set AirSim host address (Windows IP from WSL2)
export PX4_SIM_HOST_ADDR=172.28.0.1

# Start PX4 SITL — none_iris = AirSim provides physics, no jMAVSim
cd ~/PX4-Autopilot
make px4_sitl_default none_iris
```

Wait for the connection confirmation:

```
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
INFO  [commander] Ready for takeoff!
```

### Step 3: Set PX4 Parameters

In the PX4 console (`pxh>` prompt in the same terminal):

```bash
param set COM_RC_IN_MODE 4
param set FD_FAIL_R 400
param set FD_FAIL_P 400
param set MC_ROLLRATE_MAX 1800
param set COM_OF_LOSS_T 5
param save
```

> ⚠️ Run `param save` every session. Parameters reset on PX4 restart unless saved.

### Step 4: Start MAVROS in a New WSL2 Terminal

```bash
source /opt/ros/noetic/setup.bash

roslaunch mavros px4.launch \
  fcu_url:="udp://:14550@127.0.0.1:14555" \
  tgt_system:=1 \
  tgt_component:=1
```

Successful connection output:

```
[ INFO] FCU URL: udp://:14550@127.0.0.1:14555
[ INFO] MAVROS started. MY ID 1.240, TARGET ID 1.1
[ INFO] VER: 1.1: Capabilities...
```

> ⚠️ `tgt_system:=1 tgt_component:=1` is **required**. Without these flags MAVROS reports `MODE: Unsupported FCU` and cannot control the drone.

### Step 5: Run the Flip Script in a New WSL2 Terminal

```bash
source /opt/ros/noetic/setup.bash
python3 drone_sitl_flip.py
```

---

## 6. Flip Script

### 6.1 How It Works

The script is based on the [Clover drone flip algorithm](https://gist.github.com/okalachev/d9cb1769b1db775525e8afb602c3f2c0) by okalachev, adapted for direct MAVROS `AttitudeTarget` body rate control.

The critical insight: using `type_mask=128` (`IGNORE_ATTITUDE`) tells PX4 to use body angular rates directly instead of an attitude quaternion setpoint.

### 6.2 Critical Design Decisions

| Decision | Why It Matters |
|----------|----------------|
| **Background publisher thread** | PX4 exits OFFBOARD mode if no setpoint received for 500ms. A background thread publishes at 50Hz continuously — even during `rospy.sleep()` calls. Without this the drone falls during flip. |
| **`type_mask = 128`** | `IGNORE_ATTITUDE` flag. Tells PX4 to use `body_rate` values, not the quaternion. `type_mask=7` (wrong value) means *ignore the rates* — drone gets zero rate commands and falls. |
| **`sleep_publishing()` function** | Replaces `rospy.sleep()` — loops with 10ms sleeps so the background thread keeps publishing. |
| **`wait_until_stable()`** | Checks `abs(roll) < 5°` AND altitude recovered before second flip. Prevents residual angular velocity causing second flip to spin multiple times. |
| **Clover flip logic** | Thrust bump → roll at 30 rad/s → monitor until `abs(roll) > 90°` → counter-roll at -50 rad/s for 0.15s. |

### 6.3 Flip Phase Sequence

| Phase | Command | Description |
|-------|---------|-------------|
| Thrust bump | `roll=0, thrust=1.0, duration=0.2s` | Gain altitude energy before flip |
| Roll rate | `roll=30 rad/s, thrust=0.2` | Max roll rate. Monitor until `abs(roll) > 90°` |
| Counter-roll | `roll=-50 rad/s, thrust=0.8, duration=0.15s` | Stop the rotation |
| Recovery | Position setpoint `z=start_z` | Return to pre-flip altitude |

### 6.4 Full Script — `drone_sitl_flip.py`

```python
#!/usr/bin/env python3
import rospy
import math
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# ── State callbacks ───────────────────────────────────────────────────────────
current_state = State()
current_pose  = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

rospy.init_node('flip_node')
state_sub = rospy.Subscriber('mavros/state', State, state_cb)
pose_sub  = rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_cb)
pos_pub   = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
att_pub   = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

rospy.wait_for_service('/mavros/cmd/arming')
rospy.wait_for_service('/mavros/set_mode')
arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

# ── Shared setpoint state (thread-safe) ──────────────────────────────────────
setpoint_lock = threading.Lock()
current_setpoint = {
    'mode': 'pos',
    'x': 0.0, 'y': 0.0, 'z': 3.0,
    'roll_rate': 0.0, 'pitch_rate': 0.0, 'yaw_rate': 0.0,
    'thrust': 0.5
}

def update_pos(x, y, z):
    with setpoint_lock:
        current_setpoint.update({'mode': 'pos', 'x': x, 'y': y, 'z': z})

def update_rate(roll_rate, pitch_rate, yaw_rate, thrust):
    with setpoint_lock:
        current_setpoint.update({
            'mode': 'rate',
            'roll_rate': roll_rate,
            'pitch_rate': pitch_rate,
            'yaw_rate': yaw_rate,
            'thrust': thrust
        })

# ── Background publisher — NEVER stops ───────────────────────────────────────
# Keeps PX4 in OFFBOARD mode even during rospy.sleep() calls
def bg_publish():
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        with setpoint_lock:
            sp = dict(current_setpoint)
        if sp['mode'] == 'pos':
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = sp['x']
            msg.pose.position.y = sp['y']
            msg.pose.position.z = sp['z']
            msg.pose.orientation.w = 1.0
            pos_pub.publish(msg)
        else:
            msg = AttitudeTarget()
            msg.header.stamp  = rospy.Time.now()
            msg.type_mask     = 128   # IGNORE_ATTITUDE: use body rates + thrust
            msg.body_rate.x   = sp['roll_rate']
            msg.body_rate.y   = sp['pitch_rate']
            msg.body_rate.z   = sp['yaw_rate']
            msg.thrust        = sp['thrust']
            att_pub.publish(msg)
        r.sleep()

threading.Thread(target=bg_publish, daemon=True).start()

# ── Helpers ───────────────────────────────────────────────────────────────────
def get_roll():
    o = current_pose.pose.orientation
    return math.atan2(
        2*(o.w*o.x + o.y*o.z),
        1 - 2*(o.x**2 + o.y**2)
    )

def get_pos():
    p = current_pose.pose.position
    return p.x, p.y, p.z

def sleep_publishing(seconds):
    """Sleep without blocking the background publisher."""
    end = rospy.Time.now() + rospy.Duration(seconds)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        rospy.sleep(0.01)

def wait_until_stable(target_z=15.0, roll_threshold=5.0, timeout=30.0):
    """
    Wait until drone is level AND at target altitude.
    Prevents second flip starting with residual angular velocity.
    """
    rospy.loginfo(f"Waiting for stability (roll<{roll_threshold}deg, z>{target_z-2:.0f}m)...")
    update_pos(0, 0, target_z)
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        roll_deg = math.degrees(get_roll())
        x, y, z  = get_pos()
        rospy.loginfo(f"  Stability check: roll={roll_deg:.1f}deg  z={z:.2f}")
        if abs(roll_deg) < roll_threshold and z > target_z - 2.0:
            rospy.loginfo("Stable! Ready for next flip.")
            return True
        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("Stability timeout — proceeding anyway.")
            return False
        rospy.sleep(0.3)

# ── Flip function (Clover algorithm adapted for MAVROS) ──────────────────────
def flip(direction='right'):
    """
    Performs a full 360° flip.
    Based on: https://gist.github.com/okalachev/d9cb1769b1db775525e8afb602c3f2c0
    Translated from Clover set_rates() to MAVROS AttitudeTarget body rates.

    Key: type_mask=128 (IGNORE_ATTITUDE) tells PX4 to use body_rate not quaternion.
    """
    sx, sy, sz = get_pos()
    rospy.loginfo(f"[FLIP {direction.upper()}] Starting from z={sz:.2f}  roll={math.degrees(get_roll()):.1f}deg")

    # Phase 1: Thrust bump — gain energy before flip
    # Clover equivalent: set_rates(thrust=1), sleep(0.2)
    rospy.loginfo("  Phase 1: Thrust bump...")
    update_rate(0, 0, 0, 1.0)
    sleep_publishing(0.2)

    # Phase 2: Full roll rate
    # Clover equivalent: set_rates(roll_rate=30, thrust=0.2)
    roll_rate = 30.0 if direction == 'right' else -30.0
    rospy.loginfo(f"  Phase 2: Rolling at {roll_rate} rad/s...")
    update_rate(roll_rate, 0, 0, 0.2)

    # Monitor until flipped past 90 degrees
    # Clover equivalent: while True: if abs(telem.roll) > PI/2: break
    timeout = rospy.Time.now() + rospy.Duration(3.0)
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        roll = get_roll()
        rospy.loginfo(f"  roll={math.degrees(roll):.1f}deg")
        if abs(roll) > math.pi / 2:
            rospy.loginfo(f"  Flipped! roll={math.degrees(roll):.1f}deg")
            break
        rospy.sleep(0.01)

    # Phase 3: Counter-roll to stop rotation
    # Clover equivalent: set_rates(roll_rate=-50, thrust=0.8), sleep(0.15)
    counter_rate = -50.0 if direction == 'right' else 50.0
    rospy.loginfo(f"  Phase 3: Counter-roll at {counter_rate} rad/s...")
    update_rate(counter_rate, 0, 0, 0.8)
    sleep_publishing(0.15)

    # Phase 4: Return to start position
    # Clover equivalent: set_position(x=start.x, y=start.y, z=start.z)
    rospy.loginfo(f"  Phase 4: Recovering to z={sz:.2f}...")
    update_pos(sx, sy, sz)
    sleep_publishing(3.0)

    rospy.loginfo(f"[FLIP {direction.upper()}] Done! roll={math.degrees(get_roll()):.1f}deg")

# ── Main ──────────────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not current_state.connected:
    rospy.sleep(0.1)
rospy.loginfo("Connected!")

# Pre-stream setpoints — MANDATORY before requesting OFFBOARD mode
# PX4 rejects OFFBOARD switch if setpoints are not already streaming
rospy.loginfo("Pre-streaming setpoints for 5 seconds...")
update_pos(0, 0, 3)
sleep_publishing(5.0)

# Setup OFFBOARD + ARM requests
offb = SetModeRequest()
offb.custom_mode = 'OFFBOARD'
arm = CommandBoolRequest()
arm.value = True
last_req    = rospy.Time.now()
rate        = rospy.Rate(20)
phase       = "TAKEOFF"
phase_start = rospy.Time.now()

rospy.loginfo("Starting flight...")

while not rospy.is_shutdown():

    # Keep OFFBOARD mode active
    if current_state.mode != "OFFBOARD" and \
       (rospy.Time.now() - last_req) > rospy.Duration(5.0):
        if set_mode_client.call(offb).mode_sent:
            rospy.loginfo("OFFBOARD enabled!")
        last_req = rospy.Time.now()

    # Keep armed
    elif not current_state.armed and \
         (rospy.Time.now() - last_req) > rospy.Duration(5.0):
        if arming_client.call(arm).success:
            rospy.loginfo("Armed!")
        last_req = rospy.Time.now()

    x, y, z  = get_pos()
    roll_deg = math.degrees(get_roll())

    if phase == "TAKEOFF":
        update_pos(0, 0, 3)
        if current_state.armed and z > 2.5:
            rospy.loginfo(f"Takeoff complete. z={z:.2f}")
            phase = "CLIMB"

    elif phase == "CLIMB":
        update_pos(0, 0, 15)
        if z > 13.0:
            rospy.loginfo(f"Flip altitude reached. z={z:.2f}")
            phase = "STABILIZE_1"
            phase_start = rospy.Time.now()

    elif phase == "STABILIZE_1":
        update_pos(0, 0, 15)
        if (rospy.Time.now() - phase_start).to_sec() > 3.0:
            rospy.loginfo("Stable. Executing RIGHT flip...")
            phase = "FLIP_RIGHT"

    elif phase == "FLIP_RIGHT":
        flip('right')
        phase = "STABILIZE_2"

    elif phase == "STABILIZE_2":
        # Confirmed stability check — not just time delay
        wait_until_stable(target_z=15.0, roll_threshold=5.0, timeout=30.0)
        rospy.loginfo("Executing LEFT flip...")
        phase = "FLIP_LEFT"

    elif phase == "FLIP_LEFT":
        flip('left')
        phase = "DONE"

    elif phase == "DONE":
        update_pos(0, 0, 3)
        rospy.loginfo(f"All flips complete! z={z:.2f}  roll={roll_deg:.1f}deg")
        sleep_publishing(5.0)
        break

    rate.sleep()

rospy.loginfo("Script complete.")
```

---

## 7. Troubleshooting

### 7.1 Common Errors and Fixes

| Error | Fix |
|-------|-----|
| `poll timeout 0, 111` | AirSim not running or wrong `LocalHostIp`. Start AirSim FIRST then PX4. |
| `Simulator connected then immediately disconnects` | `LockStep` mismatch. Ensure `"ClockType": "SteppableClock"` in settings.json. |
| `MODE: Unsupported FCU` | MAVROS component ID mismatch. Use `tgt_system:=1 tgt_component:=1` in MAVROS launch. |
| `Address already in use (port 14540)` | Kill all processes: `killall -9 Blocks px4 && pkill -9 -f mavros` |
| `AirSim crash: parameter LPE_LAT not found` | v1.14.3 uses EKF2 not LPE. Remove `LPE_LAT` and `LPE_LON` from settings.json Parameters block entirely. |
| `Drone falls during flip` | `type_mask` was wrong. Must be `128` (`IGNORE_ATTITUDE`). Also ensure background publisher thread is running. |
| `Flip triggers failsafe` | `FD_FAIL_R` and `FD_FAIL_P` not set. Run `param set FD_FAIL_R 400` and `param save` in PX4 console. |
| `Second flip spins multiple times` | Residual angular velocity. `wait_until_stable()` checks `roll < 5deg` before second flip. |
| `Preflight Fail: ekf2 missing data` | **Normal transient during startup** — EKF2 hasn't received enough sensor data from AirSim yet. Wait 5–10 seconds. It clears automatically once AirSim starts streaming IMU/GPS/barometer data. If it persists, see fix below. |
| `Preflight check: FAILED` | EKF2 not initialized. Check `ekf2 status` in PX4 console. |
| `Drone arms but does not take off` | OFFBOARD mode dropped. Check `rostopic hz /mavros/setpoint_position/local` — must be `>2Hz`. |
| `guided: False in mavros/state` | Pre-stream setpoints for 5 seconds BEFORE requesting OFFBOARD. |

### 7.2 Diagnostic Commands

```bash
# Check MAVROS connection state
rostopic echo /mavros/state -n 1

# Check setpoint publishing rate (must be >2Hz for OFFBOARD)
rostopic hz /mavros/setpoint_position/local
rostopic hz /mavros/setpoint_raw/attitude

# Check what's on specific ports
sudo lsof -i :4560
sudo lsof -i :14550

# Kill all related processes cleanly
killall -9 Blocks px4 PX4
pkill -9 -f mavros
pkill -9 -f flip

# PX4 console diagnostics
ekf2 status
commander status
commander check

# Verify WSL2 bridge IP
cat /etc/resolv.conf | grep nameserver
```

---

## 8. Key Lessons Learned

### 8.1 The `type_mask` Problem (Most Critical)

`type_mask` is a bitmask where **1 = ignore this field**:

| Bit | Value | Field |
|-----|-------|-------|
| 0 | 1 | Ignore body roll rate |
| 1 | 2 | Ignore body pitch rate |
| 2 | 4 | Ignore body yaw rate |
| 7 | 128 | Ignore attitude quaternion |

`type_mask = 7` (`0b00000111`) = bits 0,1,2 set = **ignore all body rates** → PX4 receives zero rate commands → drone falls.

`type_mask = 128` = bit 7 set = **ignore attitude quaternion, use body rates** → flip works.

### 8.2 The Background Publisher Problem

PX4 has a 500ms offboard timeout. Any `rospy.sleep()` call blocks the main thread and stops setpoint publishing, causing PX4 to exit OFFBOARD and land.

**Solution:** A background daemon thread publishes at 50Hz continuously. The `sleep_publishing()` helper sleeps in 10ms increments without blocking the daemon thread.

```python
# WRONG — blocks publishing for 0.2 seconds
rospy.sleep(0.2)   # PX4 exits OFFBOARD → drone lands

# CORRECT — main thread sleeps but daemon thread keeps publishing
sleep_publishing(0.2)
```

### 8.3 The Stability Problem Between Flips

After the first flip the drone has residual angular velocity. Starting the second flip immediately causes it to spin multiple times. `wait_until_stable()` confirms `abs(roll) < 5°` AND altitude is recovered — this is a verified check, not a time delay.

### 8.4 LPE vs EKF2 on PX4 v1.14.3

PX4 v1.14.3 uses **EKF2** as the position estimator. The `LPE_LAT` and `LPE_LON` parameters do not exist. Setting them via settings.json crashes AirSim:

```
terminating with uncaught exception: Error: parameter name 'LPE_LAT' was not found
Signal 6 caught — Aborted (core dumped)
```

Home location in EKF2 is set automatically from GPS. No manual parameter needed.

### 8.5 MAVROS Component ID

Without `tgt_system:=1 tgt_component:=1`, MAVROS targets component ID 240 which does not match PX4 SITL's expected ID, resulting in:

```
[ERROR] MODE: Unsupported FCU
mode_sent: False
```

### 8.6 "Preflight Fail: ekf2 missing data" — What It Means

This warning appears in AirSim and the PX4 console during the first few seconds of startup. It is **expected and normal** — it does NOT indicate a problem.

**What EKF2 is:** The Extended Kalman Filter 2 is PX4's position and attitude estimator. It fuses data from IMU (accelerometer, gyroscope), GPS, barometer, and magnetometer to compute the drone's position, velocity, and orientation. Without valid EKF2 output, PX4 refuses to arm for safety.

**Why it appears at startup:** When PX4 first connects to AirSim, EKF2 needs a few seconds to:
1. Receive enough sensor samples from AirSim (IMU + barometer)
2. Converge its state estimate to a stable solution
3. Get a GPS lock from AirSim's simulated GPS
4. Set a home position

During this initialization window — typically 3–10 seconds — EKF2 reports "missing data" because it hasn't received enough samples yet to be confident in its estimate.

**What you see in your screenshot:**

```
Preflight Fail: ekf2 missing data     ← transient, appears first
Got GPS lock                          ← EKF2 received GPS data
Got GPS Home Location                 ← Home position set, EKF2 happy
```

The warning clears automatically once GPS lock is achieved. It is not an error — it is a status message showing EKF2 is initializing.

**If it persists beyond 30 seconds** — that is a real problem. Causes:

| Cause | Fix |
|-------|-----|
| AirSim not sending sensor data | Check AirSim terminal for connection errors |
| LockStep timing mismatch | Ensure `"ClockType": "SteppableClock"` in settings.json |
| Barometer noise too high | `PressureFactorSigma: 0.0001825` must be in settings.json |
| Sensors not defined | Add explicit `Barometer`, `Gps`, `Imu` blocks to settings.json |

**Diagnostic command in PX4 console:**

```bash
ekf2 status
```

Shows EKF2 internal state — should show "Local position estimate valid" once initialized.

### 8.6 OFFBOARD Pre-Streaming Requirement

PX4 rejects OFFBOARD mode switch if setpoints are not already streaming. The correct sequence is:

```
1. Start streaming setpoints (100 iterations at 20Hz = 5 seconds)
2. Request OFFBOARD mode switch
3. Request arming
4. Begin flight
```

Requesting OFFBOARD before streaming always results in rejection.

---

## 9. Next Steps: HITL

> Note: HITL testing uses a real Pixhawk running **PX4 firmware v1.9.2** (not v1.14.3). This is a separate setup documented elsewhere.

### 9.1 Key Differences: SITL vs HITL

| Aspect | SITL (this guide) | HITL (future) |
|--------|-------------------|----------------|
| PX4 version | v1.14.3 | v1.9.2 |
| PX4 runs on | WSL2 laptop | Real Pixhawk hardware |
| Simulator connection | TCP (UseSerial: false) | Serial USB (UseSerial: true, /dev/ttyACM0) |
| `make px4_sitl` command | Required | NOT used |
| Airframe selection | Auto (none_iris) | HIL Quadrocopter X in QGC |
| `LPE_LAT / LPE_LON` | Not used (EKF2) | Not used (EKF2 on v1.9.2 too) |
| Flip timing | Works as-is | May need tuning (hardware latency) |

### 9.2 HITL settings.json Changes Required

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": true,
      "SerialPort": "/dev/ttyACM0",
      "SerialBaudRate": 115200,
      "LockStep": true,
      "Sensors": {
        "Barometer": {
          "SensorType": 1,
          "Enabled": true,
          "PressureFactorSigma": 0.0001825
        }
      },
      "Parameters": {
        "COM_ARM_WO_GPS": 1,
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_RC_IN_MODE": 4,
        "COM_OBL_RC_ACT": 1,
        "SYS_HITL": 1,
        "FD_FAIL_R": 0,
        "FD_FAIL_P": 0,
        "COM_OF_LOSS_T": 5.0
      }
    }
  }
}
```

### 9.3 HITL Startup Sequence

1. Configure Pixhawk in QGC: select **HIL Quadrocopter X** airframe
2. Start AirSim: `.\Blocks.exe -ResX=640 -ResY=480 -windowed`
3. Start MAVROS: `roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200" tgt_system:=1 tgt_component:=1`
4. Run flip script: `python3 drone_sitl_flip.py` (same script, no changes)

> No `make px4_sitl` in HITL — the Pixhawk IS the PX4 flight controller.

### 9.4 Expected HITL Tuning

- `roll_rate`: may need adjustment from 30 to 25–35 rad/s
- Counter-roll `sleep_publishing(0.15)`: may need 0.20–0.25s due to serial latency
- GPS lock takes longer with real Pixhawk sensors

---

## Appendix: Quick Reference Card

### Terminal Commands

```bash
# Windows
.\Blocks.exe -ResX=640 -ResY=480 -windowed

# WSL2 — PX4 SITL
export PX4_SIM_HOST_ADDR=172.28.0.1
cd ~/PX4-Autopilot && make px4_sitl_default none_iris

# PX4 Console (after SITL starts)
param set COM_RC_IN_MODE 4
param set FD_FAIL_R 400
param set FD_FAIL_P 400
param set MC_ROLLRATE_MAX 1800
param set COM_OF_LOSS_T 5
param save

# WSL2 — MAVROS
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14555" tgt_system:=1 tgt_component:=1

# WSL2 — Flip Script
source /opt/ros/noetic/setup.bash
python3 drone_sitl_flip.py
```

### Parameter Summary

| Parameter | Value Set | Description | All Values |
|-----------|-----------|-------------|------------|
| `COM_RC_IN_MODE` | `4` | Disable RC requirement | 0=RC required, 1=Optional, 4=Disabled |
| `FD_FAIL_R` | `400` | Disable roll failure detector | 60=default (too low), 400=>360 so disabled |
| `FD_FAIL_P` | `400` | Disable pitch failure detector | Same as FD_FAIL_R |
| `MC_ROLLRATE_MAX` | `1800` | Allow 30 rad/s flip roll rate | deg/s, must exceed commanded rate |
| `COM_OF_LOSS_T` | `5` | Offboard timeout 5 seconds | seconds before failsafe triggers |
| `NAV_RCL_ACT` | `0` | No RC loss failsafe | 0=disabled, 1=loiter, 2=RTL, 3=land |
| `NAV_DLL_ACT` | `0` | No data link loss failsafe | 0=disabled, 1=loiter, 2=RTL, 3=land |
| `COM_OBL_RC_ACT` | `1` | Hover when offboard ends | 0=pos/land, 1=hold, 2=RTL, 3=land, 4=disarm |

### type_mask Reference

| Value | Meaning | Use Case |
|-------|---------|----------|
| `128` | IGNORE_ATTITUDE — use body rates | **Flip maneuver** |
| `7` | IGNORE body rates — use attitude quaternion | Normal attitude hold |
| `0` | Use everything | Full attitude + rate control |

---

*Generated: May 2026 | PX4 v1.14.3 | ROS Noetic | Ubuntu 20.04 WSL2 | AirSim v1.8.1*

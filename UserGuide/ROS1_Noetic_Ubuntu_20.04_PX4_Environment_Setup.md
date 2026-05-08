# ROS1 Noetic Ubuntu 20.04 PX4 Environment Setup

**Author:** Lukas  
**Date:** April 28, 2026  
**Target System:** Ubuntu 20.04 (Focal Fossa) with ROS1 Noetic  
**PX4 Version:** Latest (compatible with Gazebo Classic 11)

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [System Requirements](#system-requirements)
3. [Step-by-Step Installation](#step-by-step-installation)
4. [Testing the Installation](#testing-the-installation)
5. [Integration with ROS and MAVROS](#integration-with-ros-and-mavros)
6. [OFFBOARD Mode Control](#offboard-mode-control)
7. [Understanding PX4 Flight Modes](#understanding-px4-flight-modes)
8. [Troubleshooting](#troubleshooting)
9. [Common Commands Reference](#common-commands-reference)
10. [Next Steps](#next-steps)

---

## Prerequisites

Before starting, ensure you have:

- **Ubuntu 20.04 LTS (Focal Fossa)** installed (native or WSL2)
- **ROS1 Noetic** fully installed and configured
- **Gazebo Classic 11** (comes with ROS Noetic Desktop Full)
- **MAVROS** installed (for MAVLink/ROS communication)
- **Git** installed
- **Sudo privileges**
- **Stable internet connection**

### Verify Installations

```bash
# Check ROS installation
rosversion -d
# Should output: noetic

# Check Gazebo version
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x.x

# Check MAVROS installation
rospack find mavros
# Should output: /opt/ros/noetic/share/mavros
```

### Install MAVROS (if not already installed)

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras

# Install GeographicLib datasets (required for GPS simulation)
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

---

## System Requirements

### Minimum Hardware

- **CPU:** Quad-core processor (Intel i5/i7 or AMD equivalent)
- **RAM:** 8GB (16GB recommended for smooth simulation)
- **GPU:** NVIDIA GPU recommended for hardware acceleration (optional but beneficial)
- **Disk Space:** 10GB free space for PX4 source and dependencies

### Software Requirements

- Ubuntu 20.04 LTS
- ROS Noetic Desktop Full
- Gazebo Classic 11
- Python 3.8+
- Git 2.x+

---

## Step-by-Step Installation

### Step 1: Clone PX4-Autopilot Repository

Since we're using `Ros1 Noetic` and `Gazebo classic`. Therefore, `px4 autopilot` package `v1.14 stable` is optimized for this setup. Therefore
we're going to install this version to avoid any issue during the installation.

```bash
# Navigate to home directory
cd ~

# Clone PX4-Autopilot with all submodules
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Navigate into the repository
cd PX4-Autopilot

git checkout v1.14.3
Link to this version -> https://github.com/PX4/PX4-Autopilot/blob/v1.14.3/Tools/setup/ubuntu.sh

git submodule update --init --recursive

bash ./Tools/setup/ubuntu.sh
```

**Note:** The `--recursive` flag is critical — it clones all Git submodules that PX4 depends on.

Usually avbove step is quite slow and take lots of time. YOu can speed up time the process by using below method. This is 5-10x faster than full clone!

```
# Press Ctrl+C to stop current clone

cd ~

# Remove incomplete clone
rm -rf PX4-Autopilot

# Shallow clone (MUCH faster - downloads only v1.14.3, not entire history)
git clone --depth 1 --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git

cd PX4-Autopilot

# Now get submodules (this is the slow part)
git submodule update --init --recursive

bash ./Tools/setup/ubuntu.sh

# Verify you're on v1.14.3
cd ~/PX4-Autopilot
git describe --tags
```

of you encounter an issues with `matplotlib>=3.0*` after running `ubuntu.sh`. You can do the following
```
cd ~/PX4-Autopilot

vi Tools/setup/requirements.txt

Find line that says:
matplotlib>=3.0.*

change it to:
matplotlib>=3.0

save & quit
```


---

### Step 2: Alternatively, you can ignore the `Ubuntu.sh` and Install PX4 Dependencies manually

**Important:** We will NOT use PX4's `ubuntu.sh` setup script as-is because it tries to install Gazebo Harmonic, which conflicts with Gazebo Classic 11 on Ubuntu 20.04.

```bash
# Update package lists
sudo apt-get update

# Install core build dependencies
sudo apt-get install -y \
    git make cmake ninja-build ccache astyle build-essential \
    genromfs libeigen3-dev libopencv-dev libxml2-utils \
    pkg-config protobuf-compiler rsync unzip zip

# Install GStreamer (for video streaming simulation)
sudo apt-get install -y \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav

# Install additional dependencies for Gazebo Classic
sudo apt-get install -y protobuf-compiler libeigen3-dev libopencv-dev

# Install GStreamer (for camera plugins)
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-plugins-good gstreamer1.0-libav

# Install Qt (for Gazebo GUI)
sudo apt-get install -y qtbase5-dev libqt5svg5-dev

# Install Python 3 dependencies
sudo apt-get install -y \
    python3-pip python3-dev python3-jinja2 python3-numpy \
    python3-empy python3-toml python3-packaging python3-yaml

# Install GeographicLib (needed for GPS simulation)
sudo apt-get install -y geographiclib-tools geographiclib-doc
sudo /usr/sbin/geographiclib-get-geoids egm96-5

# Install Python packages using pip3
pip3 install --user \
    kconfiglib jsonschema pyros-genmsg packaging toml \
    numpy future empy jinja2 pymavlink
```

You can consider to install the following to add from above
```
cd ~/PX4-Autopilot

# Install the essential Python packages manually
pip3 install --user \
    packaging \
    toml \
    numpy \
    pyros-genmsg \
    setuptools \
    wheel \
    'matplotlib>=3.0' \
    pyserial \
    empy \
    pyulog \
    pyyaml \
    cerberus \
    jinja2 \
    jsonschema \
    kconfiglib

# Now continue with the rest of setup
sudo apt update
sudo apt install -y \
    astyle \
    build-essential \
    ccache \
    cmake \
    cppcheck \
    file \
    g++ \
    gcc \
    gdb \
    git \
    lcov \
    make \
    ninja-build \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    rsync \
    shellcheck \
    unzip \
    wget \
    xsltproc \
    zip

# Install additional dependencies for Gazebo Classic
sudo apt-get install -y protobuf-compiler libeigen3-dev libopencv-dev

# Install GStreamer (for camera plugins)
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-plugins-good gstreamer1.0-libav

# Install Qt (for Gazebo GUI)
sudo apt-get install -y qtbase5-dev libqt5svg5-dev

```
---

### Step 3: Configure Environment Variables

**For PX4-only installation:**

```bash
nano ~/.bashrc

# Add these lines at the end:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11

# Save (Ctrl+X, Y, Enter) and apply
source ~/.bashrc
```

**If you have ArduPilot Gazebo installed (from ArduPilot userguide):**

This allows both ArduPilot and PX4 to coexist safely:

```bash
nano ~/.bashrc

# Add these lines at the end:
export GAZEBO_RESOURCE_PATH="/home/jlukas/ardupilot_gazebo":"/home/jlukas/ardupilot_gazebo/worlds":"/usr/share/gazebo-11"
export GAZEBO_MODEL_PATH="/home/jlukas/ardupilot_gazebo/models":"$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models":"/usr/share/gazebo-11/models"

# Save and apply
source ~/.bashrc
```

**Important:** Do NOT add `GAZEBO_PLUGIN_PATH` to `.bashrc` — PX4 sets this automatically during runtime.

---

### Step 4: Build PX4 for Gazebo Classic SITL

```bash
# Navigate to PX4-Autopilot directory
cd ~/PX4-Autopilot

# Build PX4 for Gazebo Classic (NOT Gazebo Harmonic/Garden)
make px4_sitl gazebo-classic

# This will take 5-15 minutes on first build
```

if build is fail, then do the following
```
# Clean and rebuild
cd ~/PX4-Autopilot
make distclean
make px4_sitl gazebo-classic -j$(nproc)
```

**Expected Output:**
- Compilation messages scrolling
- No critical errors (warnings are OK)
- Final message: `Build succeeded`

**Common Build Warnings (Safe to Ignore):**
- `warning: unused variable`
- `warning: deprecated conversion`

---

## Testing the Installation

### Test 1: PX4 Standalone (Without ROS)

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic

or you can try with
make px4_sitl gazebo-classic_iris

You can try different world
make px4_sitl gazebo-classic_iris__warehouse
```

You can see all available worlds as shown below
```
ls ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```

list of worlds as follow
```
baylands.world
empty.world
hippocampus.world
ksql_airport.world
mcmillan_airfield.world
safe_landing.world
sonoma_raceway.world
typhoon_h480.world
uuv_hippocampus.world
warehouse.world
yosemite.world
```

Try to build warehouse right now
```
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris__warehouse
```

Other examples
```
# Iris quad in warehouse (indoor)
make px4_sitl gazebo-classic_iris__warehouse

# Iris quad in Yosemite (mountains)
make px4_sitl gazebo-classic_iris__yosemite

# Iris quad in Baylands (waterfront)
make px4_sitl gazebo-classic_iris__baylands

# Plane in airport runway
make px4_sitl gazebo-classic_plane__ksql_airport

# Rover in Sonoma Raceway
make px4_sitl gazebo-classic_r1_rover__sonoma_raceway
```

**What Should Happen:**

1. Gazebo window opens showing an Iris quadcopter
2. PX4 console appears with prompt: `pxh>`
3. Initialization messages scroll past
4. Final message: `INFO [commander] Ready for takeoff!`

**Common warnings (safe to ignore):**
```
WARN  [health_and_arming_checks] Preflight Fail: heading estimate invalid
Node::Advertise(): Error advertising topic [/asphalt_plane/joint_cmd]
```

These are normal — the magnetometer needs time to stabilize, and the joint control error is a Gazebo GUI quirk.

### Test 2: Basic Flight Commands

In the PX4 console (`pxh>`):

```bash
# Wait 10 seconds for sensors to stabilize

# Command takeoff
commander takeoff

# The drone should lift off to ~2.5m and hover
```

**If you get "Preflight Fail: heading estimate invalid":**

```bash
# Force arm (simulation only!)
commander arm -f
commander takeoff
```

**To land:**
```bash
commander land
```

**To shutdown:**
```bash
shutdown
# Or press Ctrl+C
```

---

## Integration with ROS and MAVROS

### Launch Sequence Overview

You'll need 3 terminals running simultaneously:

1. **Terminal 1:** PX4 SITL
2. **Terminal 2:** Gazebo (optional if PX4 already launched it)
3. **Terminal 3:** MAVROS

---

### Terminal 1: Launch PX4 SITL

```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gazebo-classic
```

**Wait for:** `INFO [commander] Ready for takeoff!`

---

### Terminal 2: Launch Gazebo (Optional)

If you want to use a custom world (like `iris_world.launch`):

```bash
roslaunch gazebo_ros iris_world.launch
```

**Note:** If PX4 already launched Gazebo in Terminal 1, skip this step.

---

### Terminal 3: Launch MAVROS

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

**Expected Output:**
```
[ INFO]: FCU: PX4 Autopilot
[ INFO]: FCU: connected
[ INFO]: GCS: connected
```

---

### Verify Connection

**Terminal 4:**

```bash
# Check MAVROS state
rostopic echo /mavros/state

# Expected output:
# connected: True
# armed: False
# mode: "MANUAL"
```

**If `connected: True`** — Success! ROS, MAVROS, and PX4 are communicating.

---

## OFFBOARD Mode Control

OFFBOARD mode allows you to control the drone programmatically via ROS. This is essential for autonomous flight, RL controllers, and research applications.

### Understanding OFFBOARD Mode Requirements

**Critical Rules:**

1. **Must publish setpoints BEFORE switching to OFFBOARD mode** — PX4 will reject the mode switch if no setpoints are streaming
2. **Must publish at >2Hz continuously** — PX4 has a 500ms timeout; if setpoints stop, it will failsafe back to the previous mode
3. **Recommended to enter OFFBOARD from Position mode** — if OFFBOARD fails, the drone will hover in place instead of falling

---

### Method 1: Position Control (Go to Coordinates)

1st we need perform the following
```
commander arm
commander takeoff
```

Then in new terminal, run the following command to check what is the current `flight mode`.
```
rostopic echo /mavros/state -n 1
```

Position control tells the drone "fly to this XYZ coordinate and stay there."

**Best for:** Waypoint missions, hovering at specific locations, structured testing

#### Step-by-Step Procedure

**Terminal 4 — Start Publishing Position Setpoints:**

```bash
rostopic pub -r 20 /mavros/setpoint_position/local geometry_msgs/PoseStamped "
header:
  stamp: now
  frame_id: 'map'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

**Leave this running!** It publishes "hover at (0, 0, 2m)" at 20Hz.

---

**Terminal 5 — Wait 5-10 seconds, then switch to OFFBOARD:**

```bash
# Switch to OFFBOARD mode
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

# Expected response: mode_sent: True
```

---

Then check the state again
```
rostopic echo /mavros/state -n 1
```

**Terminal 5 — Arm the drone:**

```bash
rosservice call /mavros/cmd/arming "value: true"

# Expected response: success: True
```

**The drone should now take off to 2m and hover.**

---

Alternatively, you can start the drone using this typical approach by `arming` and `takeoff` the drone and then switch to `OFFBOARD` as follow. This method didn't required you to perform the `setpoint` like previous step.
```
commander arm
commander takeoff
```

Then in new terminal, run the following command to check what is the current `flight mode`.
```
rostopic echo /mavros/state -n 1
```

You should see below output
```
header: 
  seq: 17
  stamp: 
    secs: 1777312873
    nsecs: 352242329
  frame_id: ''
connected: True
armed: False
guided: True
manual_input: False
mode: "AUTO.LOITER <----
```

Then we can switch to `OFFBOARD mode`.
```bash
# Switch to OFFBOARD mode
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

# Expected response: mode_sent: True
```

#### Moving to Different Positions

To change position, stop the current `rostopic pub` (Ctrl+C) and immediately start a new one:

**Move Forward 2m:**
```bash
rostopic pub -r 20 /mavros/setpoint_position/local geometry_msgs/PoseStamped "
header:
  stamp: now
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 0.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

**Move Left 1m:**
```bash
rostopic pub -r 20 /mavros/setpoint_position/local geometry_msgs/PoseStamped "
header:
  stamp: now
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 1.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

**Coordinate Frame (ENU - MAVROS default):**
- **+X** = Forward (North)
- **+Y** = Left (East)
- **+Z** = Up

---

### Method 2: Velocity Control (cmd_vel)

Velocity control tells the drone "move at this speed" instead of "go to this position."

**Best for:** Manual flying, teleoperation, reactive control, joystick integration, RL controllers that output velocities

#### Understanding Velocity Control

**Position control:**
- You say "go to coordinates (2, 0, 2)"
- Drone flies there and stops
- Like GPS navigation

**Velocity control:**
- You say "move forward at 1 m/s"
- Drone keeps moving until you change the velocity
- Like using a joystick

---

#### Step-by-Step Procedure


Terminal 1 - 1st we need perform the following 
```
commander arm
commander takeoff
```

Terminal 2 - run the following command to check what is the current flight mode.
```
rostopic echo /mavros/state -n 1
```

**Terminal 3 — Start Publishing Zero Velocity (Hover):**

```bash
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

**Leave this running!**

---

**Terminal 4 — Switch to OFFBOARD and Arm:**

```bash
# Wait 5-10 seconds after starting setpoints

# Switch to OFFBOARD
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

# Expected response: mode_sent: True

# Arm (Use this only if you dodn't want to arm using commander px4)
rosservice call /mavros/cmd/arming "value: true" 
```

Then check the state again
```
rostopic echo /mavros/state -n 1
```

**Drone arms but stays on ground (because Z velocity = 0).**

---

**Terminal 4 — Take Off (Set Upward Velocity):**

Stop (Ctrl+C) and publish:

```bash
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 1.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

**Drone climbs at 1 m/s.** Watch Gazebo and when you reach desired altitude (~2m), stop it.

---

**Terminal 4 — Hover (Stop Vertical Movement):**

```bash
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

---

Alternatively, you can start the drone using this typical approach by `arming` and `takeoff` the drone and then switch to `OFFBOARD` as follow. This method didn't required you to perform the `setpoint` like previous step.
```
commander arm
commander takeoff
```

Then in new terminal, run the following command to check what is the current `flight mode`.
```
rostopic echo /mavros/state -n 1
```

You should see below output
```
header: 
  seq: 17
  stamp: 
    secs: 1777312873
    nsecs: 352242329
  frame_id: ''
connected: True
armed: False
guided: True
manual_input: False
mode: "AUTO.LOITER <----
```

Then we can switch to `OFFBOARD mode`.
```bash
# Switch to OFFBOARD mode
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

# Expected response: mode_sent: True
```

#### Velocity Control Commands

Each time you want to change velocity, Ctrl+C the current command and paste a new one:

**Move Forward at 1 m/s:**
```bash
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

**Move Backward at 1 m/s:**
```bash
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

**Move Left at 1 m/s:**
```bash
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

**Move Right at 1 m/s:**
```bash
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

**Ascend at 0.5 m/s:**
```bash
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

**Descend at 0.5 m/s:**
```bash
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

**Rotate Left (Yaw) at 0.5 rad/s:**
```bash
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

**Rotate Right (Yaw) at 0.5 rad/s:**
```bash
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

**Stop/Hover:**
```bash
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

---

#### Understanding Velocity Fields

```yaml
linear:
  x: 1.0   # Forward (+) / Backward (-) in m/s
  y: 1.0   # Left (+) / Right (-) in m/s
  z: 1.0   # Up (+) / Down (-) in m/s
angular:
  x: 0.0   # Roll (rarely used)
  y: 0.0   # Pitch (rarely used)
  z: 0.5   # Yaw: rotate left (+) / right (-) in rad/s
```

**Units:**
- Linear velocity: **m/s** (meters per second)
- Angular velocity: **rad/s** (radians per second)
  - 0.5 rad/s ≈ 28°/second (slow rotation)
  - 1.0 rad/s ≈ 57°/second (moderate rotation)

---

### Method 3: Keyboard Teleoperation (Easiest for Testing)

Instead of manually typing commands, use the ROS keyboard teleop package:

#### Install (if not already installed):

```bash
sudo apt install ros-noetic-teleop-twist-keyboard
```

#### Usage:

**After drone is in OFFBOARD mode and armed:**

**Terminal 4:**

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=20.0 cmd_vel:=/mavros/setpoint_velocity/cmd_vel_unstamped
```

or run below command
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/mavros/setpoint_velocity/cmd_vel_unstamped

```

**Important:** The `_repeat_rate:=20.0` parameter ensures continuous publishing at 20Hz, which is required for OFFBOARD mode.

**Control Keys:**

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Holonomic mode (strafing) - hold Shift:
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

k : stop/hover

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease linear speed only
e/c : increase/decrease angular speed only

CTRL-C to quit
```

**Controls Breakdown:**
- **i** = Forward
- **,** = Backward
- **j** = Rotate left (yaw)
- **l** = Rotate right (yaw)
- **Shift+J** = Strafe left (sideways)
- **Shift+L** = Strafe right (sideways)
- **t** = Up
- **b** = Down
- **k** = Stop/Hover
- **u/o/m/.** = Diagonal movements

**This is the easiest way to manually fly the drone for testing!**

---

### Landing

**Method 1: Land via ROS service:**

```bash
rosservice call /mavros/cmd/land "min_pitch: 0.0
yaw: 0.0
latitude: 0.0
longitude: 0.0
altitude: 0.0"
```

**Method 2: Switch to AUTO.LAND mode:**

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.LAND'"
```

---

### Position vs Velocity: When to Use Each

**Use Position Control when:**
- Flying to specific GPS/local coordinates
- Waypoint missions
- You want "go here and stop" behavior
- Your RL agent outputs target positions

**Use Velocity Control when:**
- Manual flying / teleoperation
- Continuous movement (path following)
- Reactive control (obstacle avoidance)
- Integration with joysticks/game controllers
- Your RL agent outputs velocities (like PPO typically does)

---

## Understanding PX4 Flight Modes

PX4 supports multiple flight modes that determine how the drone responds to commands and who has control authority.

### What Does OFFBOARD Mean?

**OFFBOARD = "Off the flight controller board"**

- **Normal modes (ONBOARD):** The flight controller's internal software makes all decisions
- **OFFBOARD mode:** External computer (ROS/MAVROS) sends direct commands to the flight controller

**Analogy:**
- **Onboard modes** = Self-driving car (you say "drive to the mall", car figures out the route)
- **Offboard mode** = Remote-controlled car (you control every movement directly)

**Why OFFBOARD exists:**
- Research applications (RL controllers, custom algorithms)
- Companion computer control (Raspberry Pi, Jetson with ROS)
- External motion planning (MoveIt, custom path planners)
- Direct programmatic control for experiments

---

### PX4 Flight Modes Overview

#### Manual Modes (You Control Everything)

**MANUAL / STABILIZED**
- You control roll, pitch, yaw, throttle directly
- PX4 only stabilizes (keeps drone level)
- Not recommended for beginners

**ACRO (Acrobatic)**
- Full manual control, no stabilization
- For aerobatic flying, flips, rolls
- Experts only

---

#### Assisted Modes (PX4 Helps You)

**ALTITUDE**
- You control horizontal movement and yaw
- PX4 maintains altitude automatically
- Release sticks = hover at current altitude
- Good for beginners

**POSITION**
- You control movement, PX4 maintains position and altitude
- Release sticks = drone stops and hovers in place
- Requires GPS or motion capture system
- Best for manual flying

---

#### Autonomous Modes (PX4 Does Everything)

**AUTO.MISSION**
- Follows pre-programmed GPS waypoints
- Upload mission via QGroundControl
- Drone executes automatically

**AUTO.RTL (Return to Launch)**
- Emergency mode: flies back to takeoff point
- Automatic landing
- Triggered by low battery or signal loss

**AUTO.LAND**
- Lands at current position
- Descends vertically

**AUTO.TAKEOFF**
- Takes off to specified altitude
- Then hovers

**HOLD**
- Loiter/hover at current position
- Doesn't respond to stick inputs

---

#### Special Modes

**OFFBOARD**
- External computer (ROS/MAVROS) sends commands
- Requires continuous setpoint stream (>2Hz)
- Used for research and autonomous applications

**FOLLOW ME**
- Drone follows a GPS target (like your phone)
- Uses MAVLink GPS position messages

**ORBIT**
- Flies in circles around a point
- Keeps camera pointed at center

---

### Flight Mode Comparison Table

| Mode | Who Controls | Position Hold | Altitude Hold | GPS Required | Use Case |
|------|--------------|---------------|---------------|--------------|----------|
| MANUAL | You | ❌ | ❌ | ❌ | Expert manual flying |
| ALTITUDE | You | ❌ | ✅ | ❌ | Beginner manual flying |
| POSITION | You | ✅ | ✅ | ✅ | Easy manual flying |
| OFFBOARD | ROS/Computer | ✅ | ✅ | ❌ | Research/RL/Autonomous |
| AUTO.MISSION | PX4 | ✅ | ✅ | ✅ | Waypoint missions |
| AUTO.RTL | PX4 | ✅ | ✅ | ✅ | Emergency return |
| HOLD | PX4 | ✅ | ✅ | ✅ | Pause/loiter |

---

### How to Check Current Mode

**Via ROS/MAVROS:**

```bash
rostopic echo /mavros/state -n 1
```

**Look at the `mode` field:**

```yaml
header:
  seq: 123
  stamp: ...
connected: True
armed: False
guided: False
mode: "MANUAL"        # Current flight mode
system_status: 3
```

**Common mode values:**
- `"MANUAL"` - Manual/Stabilized mode
- `"ALTITUDE"` or `"ALTCTL"` - Altitude hold mode
- `"POSITION"` or `"POSCTL"` - Position hold mode
- `"OFFBOARD"` - Offboard control mode
- `"AUTO.RTL"` - Return to launch
- `"AUTO.LAND"` - Auto landing
- `"AUTO.MISSION"` - Mission mode

**Via PX4 Console:**

```bash
# In PX4 console (pxh>)
commander status
```

Look for the line showing current mode.

---

### How to Switch Modes

#### Via ROS/MAVROS (Recommended)

**Switch to POSITION mode:**
```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'POSITION'"
```

**Switch to OFFBOARD mode:**
```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
```

**Switch to AUTO.RTL (Return to Launch):**
```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.RTL'"
```

**Switch to AUTO.LAND:**
```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.LAND'"
```

**Switch to ALTITUDE mode:**
```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'ALTITUDE'"
```

---

#### Via PX4 Console

```bash
# In PX4 console (pxh>)

# Position control mode
commander mode posctl

# Altitude control mode
commander mode altctl

# Manual mode
commander mode manual

# Return to launch
commander mode auto:rtl

# Land
commander mode auto:land
```

---

### Safe Mode Transitions

**Recommended transitions:**
- POSITION → OFFBOARD (safest for entering OFFBOARD)
- ALTITUDE → OFFBOARD
- OFFBOARD → POSITION (safe fallback if OFFBOARD fails)

**Dangerous transitions:**
- MANUAL → OFFBOARD (drone might be unstable)
- OFFBOARD → MANUAL (requires manual control skills)

**PX4's automatic failsafe:**
- If OFFBOARD stops receiving setpoints for >500ms
- PX4 switches back to **previous mode** (usually POSITION)
- Drone hovers in place
- Prevents crashes if your ROS node dies

---

### Which Mode Should You Use?

**For RL research (PPO impact recovery):**
- ✅ **OFFBOARD** — Your RL agent sends velocity/position commands via ROS
- Provides direct control authority needed for learning

**For manual testing:**
- ✅ **POSITION** — Easy to fly, drone holds position when you release sticks
- Safe for experimenting with commands

**For emergency:**
- ✅ **AUTO.RTL** — Brings drone back and lands safely
- Use if something goes wrong

**For simulation experiments:**
- ✅ **OFFBOARD** — Full programmatic control via Python/ROS
- Essential for autonomous research

---

### Mode Switching Example Workflow

**Typical research flight sequence:**

```bash
# 1. Start in POSITION mode (safe default)
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'POSITION'"

# 2. Arm and takeoff
rosservice call /mavros/cmd/arming "value: true"
rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 2.0}"

# 3. Wait for stable hover, then start publishing setpoints
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &

# 4. Wait 5 seconds, switch to OFFBOARD
sleep 5
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

# 5. Your RL agent now controls the drone

# 6. Emergency return
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.RTL'"
```

---

## Troubleshooting

### Issue 1: "Poll Timeout" Errors When Switching to OFFBOARD

**Symptom:**
```
ERROR [simulator_mavlink] poll timeout 0, 22
```

**Cause:** PX4 can't communicate with Gazebo.

**Solution:**

1. Make sure Gazebo is actually running:
   ```bash
   ps aux | grep gazebo
   # Should show gzserver and gzclient
   ```

2. If using HEADLESS mode, make sure you launched Gazebo separately via ROS:
   ```bash
   roslaunch gazebo_ros empty_world.launch
   ```

3. Don't mix ArduPilot worlds with PX4 — use PX4's default worlds or create compatible ones.

---

### Issue 2: OFFBOARD Mode Rejected / Timeout

**Symptom:** Mode switch fails or times out immediately.

**Cause:** Not publishing setpoints before switching to OFFBOARD.

**Solution:** Always follow this sequence:

1. Start publishing setpoints (at 20Hz)
2. Wait 5-10 seconds
3. **Then** switch to OFFBOARD
4. **Then** arm

Never switch to OFFBOARD before setpoints are streaming!

---

### Issue 3: "Heading Estimate Invalid" Won't Clear

**Symptom:** PX4 refuses to arm due to magnetometer errors.

**Solution:**

```bash
# In PX4 console
commander arm -f  # Force arm (simulation only!)
```

Or wait 15-30 seconds for GPS/magnetometer to initialize.

---

### Issue 4: Gazebo Can't Find Models

**Symptom:** Error messages like `Model 'iris' not found`.

**Solution:**

Check your GAZEBO_MODEL_PATH includes PX4 models:

```bash
echo $GAZEBO_MODEL_PATH
# Should include: .../PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```

If not, add it to `.bashrc` and `source ~/.bashrc`.

---

### Issue 5: MAVROS Can't Connect

**Symptom:** `rostopic echo /mavros/state` shows `connected: False`.

**Solution:**

Check PX4 MAVLink status:

```bash
# In PX4 console
mavlink status
# Should show UDP connections on ports 14540, 14557
```

Verify MAVROS is using correct URL:
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

---

### Issue 6: Build Fails with "Submodule Error"

**Symptom:** Error during `make` about missing submodules.

**Solution:**

```bash
cd ~/PX4-Autopilot
git submodule update --init --recursive
make clean
make px4_sitl gazebo-classic
```

---

### Issue 7: GPU Not Being Used (WSL2)

**Symptom:** Low FPS, `nvidia-smi` shows no Gazebo process.

**Solution:**

Your WSL2 OpenGL version is likely capped at 3.1. Check:

```bash
glxinfo -B | grep "OpenGL version"
```

If showing 3.1, Gazebo is already using GPU but limited by OpenGL version. This is a WSL2 limitation, not a configuration issue. Performance is acceptable for simulation work.

---

## Common Commands Reference

### PX4 SITL Control

```bash
# Launch PX4 with Gazebo GUI
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic

# Launch PX4 headless (no Gazebo auto-launch)
HEADLESS=1 make px4_sitl gazebo-classic

# Clean build (if something breaks)
make distclean
make px4_sitl gazebo-classic
```

### PX4 Console Commands

```bash
# Arm
commander arm

# Force arm (ignore preflight checks - simulation only!)
commander arm -f

# Takeoff
commander takeoff

# Land
commander land

# Check sensor status
commander status

# Check MAVLink connections
mavlink status

# Shutdown PX4
shutdown
```

### ROS/MAVROS Commands

```bash
# Check connection
rostopic echo /mavros/state

# List all MAVROS topics
rostopic list | grep mavros

# Arm via ROS
rosservice call /mavros/cmd/arming "value: true"

# Set mode to OFFBOARD
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

# Takeoff (direct command, not via OFFBOARD)
rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 2.0}"

# Land
rosservice call /mavros/cmd/land "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
```

### Monitoring Commands

```bash
# Monitor IMU data
rostopic echo /mavros/imu/data

# Monitor local position
rostopic echo /mavros/local_position/pose

# Monitor battery (simulated)
rostopic echo /mavros/battery

# Monitor GPS (simulated)
rostopic echo /mavros/global_position/global

# Check setpoint publishing rate
rostopic hz /mavros/setpoint_position/local
# Or for velocity:
rostopic hz /mavros/setpoint_velocity/cmd_vel_unstamped

# Record flight data
rosbag record /mavros/imu/data /mavros/local_position/pose /mavros/state -O flight_test.bag
```

---

## Next Steps

### For Research Work (PPO + Adaptive PID)

1. **Create a custom Gazebo world** with obstacles/impact scenarios
2. **Integrate your RL pipeline** (PPO training loop publishing to `/mavros/setpoint_velocity/cmd_vel_unstamped`)
3. **Access PX4's EKF state** via `/mavros/local_position/*` topics for your state observations
4. **Implement custom controllers** via MAVROS OFFBOARD mode
5. **Log data** using rosbag for analysis and paper figures

### Comparing ArduPilot vs PX4

Since you have both installed, run them sequentially (never simultaneously):

**ArduPilot Session:**

```bash
# Terminal 1
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --console

# Terminal 2
roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@14555"

# Run your tests, collect rosbag data
```

**PX4 Session (separate day/run):**

```bash
# Terminal 1
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic

# Terminal 2
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# Run same tests, collect rosbag data
```

**Never run both simultaneously** — they'll conflict on MAVLink ports and Gazebo.

---

## Quick Reference: Complete OFFBOARD Workflow

### Using Velocity Control (Recommended for RL)

```bash
# Terminal 1: PX4
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gazebo-classic

# Terminal 2: Gazebo
roslaunch gazebo_ros iris_world.launch

# Terminal 3: MAVROS
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# Terminal 4: Start publishing zero velocity
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Terminal 5: Wait 5-10 sec, switch to OFFBOARD
sleep 10
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

# Arm
rosservice call /mavros/cmd/arming "value: true"

# Terminal 4: Ctrl+C, then takeoff (ascend at 1 m/s)
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Wait ~2 seconds (reaches 2m), then Ctrl+C and hover
rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# OR use keyboard control instead:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=20.0 cmd_vel:=/mavros/setpoint_velocity/cmd_vel_unstamped
```

---

## Additional Resources

### Official Documentation

- **PX4 User Guide:** https://docs.px4.io/
- **PX4 Developer Guide:** https://dev.px4.io/
- **MAVROS Documentation:** http://wiki.ros.org/mavros
- **MAVROS OFFBOARD Tutorial:** https://docs.px4.io/main/en/ros/mavros_offboard_python
- **Gazebo Classic Tutorials:** http://gazebosim.org/tutorials

### Useful GitHub Repositories

- **PX4-Autopilot:** https://github.com/PX4/PX4-Autopilot
- **MAVROS:** https://github.com/mavlink/mavros
- **PX4 ROS Examples:** https://github.com/PX4/px4_ros_com

### Community Support

- **PX4 Discuss Forum:** https://discuss.px4.io/
- **ROS Answers:** https://answers.ros.org/
- **PX4 Slack:** https://px4.io/slack/

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-04-27 | Initial guide based on successful Ubuntu 20.04 + ROS Noetic + PX4 installation |
| 2.0 | 2026-04-28 | Added OFFBOARD mode control section with position control, velocity control (cmd_vel), and keyboard teleoperation methods |
| 3.0 | 2026-04-28 | Added Understanding PX4 Flight Modes section with mode comparison table, explanations, and mode switching commands |

---

## License

This guide is provided as-is for educational and research purposes.

---

**End of Guide**

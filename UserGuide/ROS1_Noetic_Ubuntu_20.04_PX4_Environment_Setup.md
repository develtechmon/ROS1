# ROS1 Noetic Ubuntu 20.04 PX4 Environment Setup
 
**Author:** Lukas  
**Date:** April 27, 2026  
**Target System:** Ubuntu 20.04 (Focal Fossa) with ROS1 Noetic  
**PX4 Version:** Latest (compatible with Gazebo Classic 11)
 
---
 
## Table of Contents
 
1. [Prerequisites](#prerequisites)
2. [System Requirements](#system-requirements)
3. [Installation Overview](#installation-overview)
4. [Step-by-Step Installation](#step-by-step-installation)
5. [Testing the Installation](#testing-the-installation)
6. [Integration with ROS and MAVROS](#integration-with-ros-and-mavros)
7. [Troubleshooting](#troubleshooting)
8. [Common Commands Reference](#common-commands-reference)
9. [Next Steps](#next-steps)
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
### Verify ROS Noetic Installation
 
```bash
# Check ROS installation
rosversion -d
# Should output: noetic
 
# Check Gazebo version
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x.x
```
 
### Verify MAVROS Installation
 
```bash
rospack find mavros
# Should output: /opt/ros/noetic/share/mavros
```
 
If MAVROS is not installed:
 
```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
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
 
## Installation Overview
 
The installation process involves:
 
1. Cloning the PX4-Autopilot repository
2. Installing PX4 dependencies (WITHOUT conflicting Gazebo versions)
3. Building PX4 for Software-In-The-Loop (SITL) simulation
4. Configuring environment variables
5. Testing PX4 standalone
6. Integrating with ROS/MAVROS
**Important:** We will NOT use PX4's `ubuntu.sh` setup script as-is because it tries to install Gazebo Harmonic, which conflicts with Gazebo Classic 11 on Ubuntu 20.04.
 
---
 
## Step-by-Step Installation
 
### Step 1: Clone PX4-Autopilot Repository
 
```bash
# Navigate to home directory
cd ~
 
# Clone PX4-Autopilot with all submodules
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
 
# Navigate into the repository
cd PX4-Autopilot
 
# Verify the clone was successful
ls -la
# You should see directories like: src/, msg/, Tools/, platforms/, etc.
```
 
**Note:** The `--recursive` flag is critical — it clones all Git submodules that PX4 depends on.
 
---
 
### Step 2: Install PX4 Dependencies (Manual Method)
 
Instead of running the full `ubuntu.sh` script (which tries to install incompatible Gazebo), we install dependencies manually.
 
```bash
# Update package lists
sudo apt-get update
 
# Install core build dependencies
sudo apt-get install -y \
    git \
    make \
    cmake \
    ninja-build \
    ccache \
    astyle \
    build-essential \
    genromfs \
    libeigen3-dev \
    libopencv-dev \
    libxml2-utils \
    pkg-config \
    protobuf-compiler \
    rsync \
    unzip \
    zip
 
# Install GStreamer (for video streaming simulation)
sudo apt-get install -y \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav
 
# Install Python 3 dependencies
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-jinja2 \
    python3-numpy \
    python3-empy \
    python3-toml \
    python3-packaging \
    python3-yaml
 
# Install GeographicLib (needed for GPS simulation)
sudo apt-get install -y \
    geographiclib-tools \
    geographiclib-doc
 
# Download GeographicLib geoid datasets
sudo /usr/sbin/geographiclib-get-geoids egm96-5
```
 
---
 
### Step 3: Install Python Packages for PX4
 
```bash
# Install Python packages using pip3
pip3 install --user \
    kconfiglib \
    jsonschema \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    future \
    empy \
    jinja2 \
    pymavlink
 
# Verify installation
pip3 list | grep -E "kconfiglib|pymavlink|jinja2"
```
 
---
 
### Step 4: Configure Environment Variables
 
Add Gazebo paths so PX4 can find Gazebo Classic 11 plugins and models:
 
```bash
# Open .bashrc
nano ~/.bashrc
 
# Add the following lines at the end:
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11
 
# Save and exit (Ctrl+X, then Y, then Enter)
 
# Apply changes
source ~/.bashrc
```

If you install `ardupilot_gazebo` from my other userguide `ros1_noetic_ubuntu_20_Gazebo_SITL_setup.md`. You can append `px4` gazebo environments as follow. This is safe method to combine `ardupilot` and `px4` gazebo environments
```
bash
# Open .bashrc
nano ~/.bashrc

# Gazebo paths for both ArduPilot and PX4
export GAZEBO_RESOURCE_PATH="/home/jlukas/ardupilot_gazebo":"/home/jlukas/ardupilot_gazebo/worlds":"/usr/share/gazebo-11"
export GAZEBO_MODEL_PATH="/home/jlukas/ardupilot_gazebo/models":"$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models":"/usr/share/gazebo-11/models"
export GAZEBO_PLUGIN_PATH="$HOME/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic":"/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"

# Save and exit (Ctrl+X, then Y, then Enter)
 
# Apply changes
source ~/.bashrc
```
 
---
 
### Step 5: Build PX4 for Gazebo Classic SITL
 
```bash
# Navigate to PX4-Autopilot directory
cd ~/PX4-Autopilot
 
# Build PX4 for Gazebo Classic (NOT Gazebo Harmonic/Garden)
make px4_sitl gazebo-classic
 
# This will take 5-15 minutes on first build
# You'll see compilation progress messages
```
 
**Expected Output:**
- Compilation messages scrolling
- No critical errors (warnings are OK)
- Final message: `Build succeeded`
**Common Build Warnings (Safe to Ignore):**
- `warning: unused variable`
- `warning: deprecated conversion`
- `note: suggested alternative`
---
 
### Step 6: Test PX4 Standalone (Without ROS)
 
```bash
# Launch PX4 SITL with Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```
 
**What Should Happen:**
 
1. **Gazebo window opens** showing an Iris quadcopter on a ground plane
2. **PX4 console appears** with prompt: `pxh>`
3. **Initialization messages** scroll past (sensor checks, calibrations, etc.)
4. **Final message:** `INFO [commander] Ready for takeoff!`
**You may see these warnings (NORMAL, ignore them):**
 
```
WARN  [health_and_arming_checks] Preflight Fail: heading estimate invalid
Node::Advertise(): Error advertising topic [/asphalt_plane/joint_cmd]
```
 
These are harmless — the magnetometer needs a few seconds to stabilize, and the joint control error is a Gazebo GUI quirk.
 
---
 
### Step 7: Test Basic Flight Commands
 
In the PX4 console (`pxh>`), try these commands:
 
```bash
# Wait 10 seconds after startup for sensors to stabilize
 
# Command takeoff
commander takeoff
 
# The drone should lift off to ~2.5m and hover
```
 
**If you get "Preflight Fail: heading estimate invalid":**
 
```bash
# Force arm and takeoff (simulation only!)
commander arm -f
commander takeoff
```
 
**To land:**
 
```bash
commander land
```
 
**To shutdown PX4:**
 
```bash
shutdown
# Or just press Ctrl+C
```
 
**If the test works:** PX4 is installed correctly! Proceed to ROS integration.
 
---
 
## Integration with ROS and MAVROS
 
Now we'll connect PX4 to ROS using MAVROS.
 
### Step 1: Launch PX4 SITL in Headless Mode
 
Headless mode runs PX4 without launching Gazebo automatically, so we can launch Gazebo via ROS instead.
 
**Terminal 1 - PX4 SITL:**
 
```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gazebo-classic
```
 
**Expected Output:**
- PX4 console appears (`pxh>`)
- No Gazebo window (that's correct — we'll launch it via ROS)
- Message: `Ready for takeoff!`
---
 
### Step 2: Launch Gazebo via ROS
 
**Terminal 2 - Gazebo GUI:**
 
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash
 
# Launch Iris World Gazebo with empty world
roslaunch gazebo_ros iris_world.launch
```
 
**Expected Output:**
- Gazebo window opens
- You should see the Iris quadcopter in the world
- No errors related to model loading
---
 
### Step 3: Launch MAVROS
 
**Terminal 3 - MAVROS:**
 
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash
 
# Launch MAVROS with PX4-specific configuration
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
 
**Expected Output:**
 
```
[ INFO] [timestamp]: FCU: PX4 Autopilot
[ INFO] [timestamp]: FCU: connected
[ INFO] [timestamp]: GCS: connected
```
 
---
 
### Step 4: Verify ROS Connection
 
**Terminal 4 - ROS Topics:**
 
```bash
# Check MAVROS state
rostopic echo /mavros/state
 
# Expected output (should update continuously):
# header: 
#   seq: 123
#   stamp: ...
# connected: True
# armed: False
# guided: False
# mode: "MANUAL"
```
 
**If `connected: True`** — Success! ROS, MAVROS, and PX4 are communicating.
 
---
 
### Step 5: Test Flight via ROS
 
**Set mode to OFFBOARD and arm:**
 
```bash
# Enable OFFBOARD mode
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
 
# Arm the vehicle
rosservice call /mavros/cmd/arming "value: true"
 
# Takeoff to 2 meters
rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 2.0}"
```
 
**The drone should take off in Gazebo.**
 
**To land:**
 
```bash
rosservice call /mavros/cmd/land "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
```
 
---
 
## Troubleshooting
 
### Issue 1: "heading estimate invalid" Won't Clear
 
**Symptom:** PX4 refuses to arm due to magnetometer errors.
 
**Solution:**
 
```bash
# In PX4 console
commander arm -f  # Force arm (simulation only!)
```
 
---
 
### Issue 2: Gazebo Can't Find Models
 
**Symptom:** Error messages like `Model 'iris' not found`.
 
**Solution:**
 
```bash
# Add PX4 model paths to Gazebo
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models' >> ~/.bashrc
source ~/.bashrc
```
 
---
 
### Issue 3: MAVROS Can't Connect
 
**Symptom:** `rostopic echo /mavros/state` shows `connected: False`.
 
**Solution:**
 
Check PX4 is running and listening on correct ports:
 
```bash
# In PX4 console, check MAVLink status
mavlink status
 
# Should show UDP connections on ports 14540, 14557
```
 
If not, restart PX4 with:
 
```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gazebo-classic
```
 
---
 
### Issue 4: Build Fails with "Submodule Error"
 
**Symptom:** Error during `make` about missing submodules.
 
**Solution:**
 
```bash
cd ~/PX4-Autopilot
git submodule update --init --recursive
make clean
make px4_sitl gazebo-classic
```
 
---
 
### Issue 5: Python Package Import Errors
 
**Symptom:** `ModuleNotFoundError: No module named 'kconfiglib'` or similar.
 
**Solution:**
 
```bash
# Reinstall Python packages
pip3 install --user --upgrade \
    kconfiglib jsonschema pyros-genmsg packaging toml numpy future empy jinja2 pymavlink
```
 
---
 
### Issue 6: GPU Not Being Used (WSL2 Users)
 
**Symptom:** Low FPS, high CPU usage, `nvidia-smi` shows no Gazebo process.
 
**Solution:**
 
You're likely hitting OpenGL version limitations. See your `glxinfo` output — if OpenGL version is 3.1, force Gazebo to use OGRE1:
 
```bash
# Launch Gazebo with OGRE1 renderer (compatible with OpenGL 3.1)
# This won't affect PX4, only improves rendering performance
export GAZEBO_RENDER_ENGINE=ogre
roslaunch gazebo_ros empty_world.launch
```
 
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
make clean
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
 
# Set mode
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
 
# Takeoff
rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 2.0}"
 
# Land
rosservice call /mavros/cmd/land "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
```
 
### Useful Monitoring Commands
 
```bash
# Monitor IMU data
rostopic echo /mavros/imu/data
 
# Monitor local position
rostopic echo /mavros/local_position/pose
 
# Monitor battery (simulated)
rostopic echo /mavros/battery
 
# Monitor GPS (simulated)
rostopic echo /mavros/global_position/global
 
# Record flight data
rosbag record /mavros/imu/data /mavros/local_position/pose /mavros/state -O flight_test.bag
```
 
---
 
## Next Steps
 
### For Research Work (PPO + Adaptive PID)
 
1. **Create a custom Gazebo world** with obstacles/impact scenarios
2. **Integrate your RL pipeline** (PPO training loop)
3. **Access PX4's EKF state** via `/mavros/local_position/*` topics
4. **Implement custom controllers** via MAVROS offboard mode
5. **Log data** using rosbag for analysis
### Comparing ArduPilot vs PX4
 
Since you have both installed:
 
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
HEADLESS=1 make px4_sitl gazebo-classic
 
# Terminal 2
roslaunch gazebo_ros empty_world.launch
 
# Terminal 3
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
 
# Run same tests, collect rosbag data
```
 
**Never run both simultaneously** — they'll conflict on ports.
 
---
 
## Additional Resources
 
### Official Documentation
 
- **PX4 User Guide:** https://docs.px4.io/
- **PX4 Developer Guide:** https://dev.px4.io/
- **MAVROS Documentation:** http://wiki.ros.org/mavros
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
 
---
 
## License
 
This guide is provided as-is for educational and research purposes.
 
---
 
**End of Guide**

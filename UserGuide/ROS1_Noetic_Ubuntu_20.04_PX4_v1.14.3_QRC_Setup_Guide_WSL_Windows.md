# ROS1 Noetic + Ubuntu 20.04 + PX4 v1.14.3 + QGroundControl Setup Guide (WSL2)

## Overview

This guide provides a complete setup for PX4 SITL simulation with Gazebo Classic, integrated with ROS1 Noetic and QGroundControl on a Windows 11 + WSL2 environment.

**What you'll achieve:**
- PX4 Autopilot v1.14.3 running in WSL2 (Ubuntu 20.04)
- Gazebo Classic simulation with various vehicle types
- QGroundControl (Windows) connected to WSL simulation
- ROS1 Noetic with MAVROS for programmatic control

**System Requirements:**
- Windows 11 with WSL2 installed
- Ubuntu 20.04 in WSL2
- 8GB RAM minimum (16GB recommended)
- 50GB free disk space

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Install PX4 Autopilot v1.14.3](#install-px4-autopilot-v1143)
3. [Install QGroundControl (Windows)](#install-qgroundcontrol-windows)
4. [Configure WSL Network Connection](#configure-wsl-network-connection)
5. [Start PX4 Gazebo Classic Simulation](#start-px4-gazebo-classic-simulation)
6. [Connect QGroundControl](#connect-qgroundcontrol)
7. [Install ROS1 Noetic and MAVROS](#install-ros1-noetic-and-mavros)
8. [Full Stack Integration](#full-stack-integration)
9. [Troubleshooting](#troubleshooting)
10. [Quick Reference](#quick-reference)

---

## Prerequisites

### Verify WSL2 Installation

Open PowerShell and check WSL version:

```powershell
wsl --list --verbose
```

You should see Ubuntu with VERSION 2.

If not installed, install WSL2:

```powershell
wsl --install
```

### Verify Ubuntu 20.04

In WSL terminal:

```bash
lsb_release -a
```

Should show: `Ubuntu 20.04.x LTS`

---

## Install PX4 Autopilot v1.14.3

### Step 1: Clone PX4 Repository

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

### Step 2: Checkout v1.14.3

```bash
git checkout v1.14.3
git submodule update --init --recursive
```

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

# Verify you're on v1.14.3
cd ~/PX4-Autopilot
git describe --tags
```

### Step 3: Fix Python Dependencies

PX4 v1.14.3 has a known dependency syntax issue. Fix it:

```bash
sed -i 's/matplotlib>=3.0.\*/matplotlib>=3.0/g' Tools/setup/requirements.txt
```

### Step 4: Install Dependencies

```bash
bash ./Tools/setup/ubuntu.sh 
```

### Step 6: Build PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic

or

# Build (fast with all cores)
make px4_sitl gazebo-classic -j$(nproc)
```

**First build takes 5-10 minutes.** Subsequent builds are faster.

### Step 7: Verify Installation

```bash
make px4_sitl gazebo-classic_iris
```

You should see:
- Gazebo Classic window opens with Iris quadcopter
- PX4 console shows startup messages
- No error messages

Press `Ctrl+C` to stop the simulation.

---

## Install QGroundControl (Windows)

### Step 1: Download QGroundControl

1. Open browser and go to: https://qgroundcontrol.com/
2. Click **Download** → **Windows**
3. Download `QGroundControl-installer.exe`

### Step 2: Install

1. Run `QGroundControl-installer.exe`
2. If Windows SmartScreen appears: **More info** → **Run anyway**
3. Follow installation wizard (accept defaults)
4. Click **Finish**

### Step 3: Launch and Allow Firewall

1. Launch QGroundControl from Start Menu
2. Windows Firewall prompt appears → Click **Allow access**
3. QGroundControl opens

**Do not configure connection yet** — we'll do that after getting the WSL IP address.

---

## Configure WSL Network Connection

### Step 1: Get WSL IP Address

Every time WSL restarts, it gets a new IP. Find it:

In WSL terminal:

```bash
ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1
```

Example output:
```
172.28.15.216
```

**Write this IP down** — you'll need it in the next step.

### Step 2: Configure QGroundControl Connection

1. Open QGroundControl
2. Click **Q icon** (top-left) → **Application Settings**
3. Click **Comm Links** tab
4. Click **Add** (bottom-right)

Configure the new link:

| Setting | Value |
|---------|-------|
| **Name** | `WSL PX4` |
| **Type** | `UDP` |
| **Automatically Connect on Start** | ☑ Checked |
| **High Latency** | ☐ Unchecked |
| **Listening Port** | `14550` |

5. Click **Add Server** under **Server Addresses**
6. Enter:
   - **Server Address:** Your WSL IP (e.g., `172.28.15.216`)
   - **Port:** `18570`
7. Click **OK** → **OK**

### Step 3: Configure Windows Firewall

Open PowerShell as **Administrator**:

```powershell
# Allow PX4 MAVLink ports
New-NetFirewallRule -DisplayName "PX4 SITL WSL" -Direction Inbound -Protocol UDP -LocalPort 18570 -Action Allow
New-NetFirewallRule -DisplayName "PX4 MAVLink" -Direction Inbound -Protocol UDP -LocalPort 14550 -Action Allow
```

### Optional: Permanent IP Solution (Mirrored Networking)

**For Windows 11 22H2 or newer only:**

Create `.wslconfig` file on Windows:

In PowerShell:

```powershell
notepad $env:USERPROFILE\.wslconfig
```

Add this content:

```ini
[wsl2]
networkingMode=mirrored
```

Save and close. Then restart WSL:

```powershell
wsl --shutdown
```

Wait 10 seconds, then reopen WSL.

**Now update QGroundControl:**
- Server Address: `127.0.0.1`
- Port: `18570`

With mirrored mode, the IP never changes — always use `localhost`.

---

## Start PX4 Gazebo Classic Simulation

### Available Vehicle Types

```bash
# Quadcopter (default)
make px4_sitl gazebo-classic_iris

# Quadcopter with optical flow sensor (for indoor/GPS-denied flight)
make px4_sitl gazebo-classic_iris_opt_flow

# Fixed-wing plane
make px4_sitl gazebo-classic_plane

# VTOL (vertical takeoff and landing)
make px4_sitl gazebo-classic_standard_vtol

# Hexacopter
make px4_sitl gazebo-classic_typhoon_h480
```

### Launch Simulation

In WSL terminal:

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

**Wait for initialization messages:**

```
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [commander] home: 47.6414680, -122.1401672, 119.99
INFO  [tone_alarm] home_set
```

When you see `home_set`, the simulation is ready.

---

## Connect QGroundControl

### Step 1: Auto-Connect

QGroundControl should **automatically connect** within 5-10 seconds after PX4 starts.

Top toolbar should show:
- **Connected** (green)
- Vehicle status: "Ready to Fly"
- GPS, battery, and telemetry indicators active

### Step 2: Manual Connect (if needed)

If not auto-connected:

1. **Q icon** → **Application Settings** → **Comm Links**
2. Select **WSL PX4**
3. Click **Connect**

### Step 3: Verify Connection

**In QGroundControl:**
- Attitude indicator shows level
- Altitude: ~0m
- Battery: 100%
- Map shows drone near Seattle, WA

**In PX4 Console (WSL):**

```bash
mavlink status
```

Should show:

```
instance #0:
        GCS link on UDP port 18570
        mode: Normal
        transport protocol: UDP
        partner IP: 172.28.x.x (your Windows IP)
```

### Step 4: Test Flight

1. Click **Disarmed** → **Armed**
2. Click **Action** → **Takeoff**
3. Drone ascends to 2.5m in Gazebo
4. QGC altitude increases

**Success!** QGroundControl is connected.

---

## Install ROS1 Noetic and MAVROS

### Step 1: Install ROS1 Noetic

In WSL terminal:

```bash
# Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS Noetic Desktop Full
sudo apt update
sudo apt install ros-noetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install MAVROS

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

### Step 3: Install GeographicLib Datasets

Required for MAVROS GPS conversions:

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh
```

### Step 4: Verify ROS Installation

```bash
rosversion -d
```

Should output: `noetic`

---

## Full Stack Integration

Run PX4, Gazebo, QGroundControl, and ROS/MAVROS together.

### Terminal 1: Start PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

Wait for `home_set` message.

### Terminal 2: Launch MAVROS

Open a new WSL terminal:

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

**Wait for connection message:**

```
[ INFO] [1234567890.123456]: CON: Got HEARTBEAT, connected.
```

### Terminal 3: Monitor ROS Topics

Open another WSL terminal:

```bash
source /opt/ros/noetic/setup.bash

# List all MAVROS topics
rostopic list

# Monitor connection state
rostopic echo /mavros/state
```

You should see:

```yaml
connected: True
armed: False
guided: False
mode: "MANUAL"
```

### Windows: QGroundControl

QGroundControl automatically connects and displays telemetry.

**All systems running simultaneously:**
- PX4 SITL (Terminal 1)
- Gazebo Classic (GUI)
- MAVROS (Terminal 2)
- QGroundControl (Windows)

---

## Troubleshooting

### QGroundControl Shows "Disconnected"

**Check 1: Verify WSL IP**

```bash
ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1
```

If different from configured IP, update QGroundControl settings.

**Check 2: Verify PX4 is Broadcasting**

In PX4 console:

```bash
mavlink status
```

Should show port 18570 active.

**Check 3: Test Windows → WSL Connectivity**

In PowerShell:

```powershell
# Replace with your WSL IP
ping 172.28.15.216
```

If ping fails, restart WSL:

```powershell
wsl --shutdown
wsl
```

**Check 4: Firewall Rules**

Verify firewall rules exist:

```powershell
Get-NetFirewallRule -DisplayName "PX4*"
```

Should show two rules. If missing, re-run firewall commands.

---

### MAVROS Won't Connect

**Check 1: Verify PX4 Onboard Port**

In PX4 console:

```bash
mavlink status
```

Should show:

```
instance #1:
        Onboard link on UDP port 14580
```

**Check 2: Check MAVROS Launch Parameters**

Verify `fcu_url` parameter:

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

The format is: `udp://[bind_host]:[bind_port]@[remote_host]:[remote_port]`

**Check 3: Monitor MAVROS Node Output**

Look for errors in the MAVROS terminal. Common issues:
- `FCU: DeviceError:serial:open: Permission denied` → Wrong port or permissions
- `bind: Address already in use` → Port 14540 already taken
- No HEARTBEAT messages → PX4 not running or wrong ports

---

### Gazebo Classic Window Doesn't Open

**Check 1: Verify X11 Display**

In WSL terminal:

```bash
echo $DISPLAY
```

If empty, set it:

```bash
export DISPLAY=:0
```

**Check 2: Check WSLg is Installed**

For Windows 11, WSLg should be installed by default. Verify:

```bash
echo $WAYLAND_DISPLAY
```

Should return something like `wayland-0`.

If not installed, update WSL:

```powershell
wsl --update
```

---

### Build Errors

**Error: `matplotlib>=3.0.*` syntax error**

Already fixed in Step 3 of PX4 installation. If you still see it:

```bash
cd ~/PX4-Autopilot
sed -i 's/matplotlib>=3.0.\*/matplotlib>=3.0/g' Tools/setup/requirements.txt
pip3 install --user matplotlib
```

**Error: `ninja: error: unknown target 'gazebo-classic_iris'`**

You're on wrong PX4 version:

```bash
cd ~/PX4-Autopilot
git checkout v1.14.3
git submodule update --init --recursive
make clean
make px4_sitl gazebo-classic_iris
```

**Error: `Gazebo not found`**

Install Gazebo Classic:

```bash
sudo apt update
sudo apt install gazebo11 libgazebo11-dev
```

---

## Quick Reference

### Common Commands

**Start PX4 Simulation:**
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

**Get WSL IP:**
```bash
ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1
```

**Launch MAVROS:**
```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

**Check MAVROS Topics:**
```bash
rostopic list
rostopic echo /mavros/state
```

**Check PX4 MAVLink Status:**
```bash
mavlink status
```

---

### Port Reference

| Port | Purpose | Direction |
|------|---------|-----------|
| 14550 | QGroundControl | PX4 → QGC |
| 14540 | MAVROS onboard | PX4 ↔ MAVROS |
| 14557 | MAVROS remote | MAVROS → PX4 |
| 18570 | QGC (WSL) | PX4 → QGC (Windows) |
| 4560 | Gazebo simulator | PX4 ↔ Gazebo |

---

### Startup Script

Save this as `~/start_px4_sim.sh`:

```bash
#!/bin/bash

# Get WSL IP
WSL_IP=$(ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)

echo "========================================"
echo "Starting PX4 SITL Simulation"
echo "========================================"
echo ""
echo "WSL IP Address: $WSL_IP"
echo ""
echo "QGroundControl Settings:"
echo "  Server: $WSL_IP"
echo "  Port: 18570"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Start PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

Make executable:

```bash
chmod +x ~/start_px4_sim.sh
```

Run:

```bash
~/start_px4_sim.sh
```

---

## Additional Resources

- **PX4 Documentation:** https://docs.px4.io/
- **QGroundControl User Guide:** https://docs.qgroundcontrol.com/
- **MAVROS Documentation:** http://wiki.ros.org/mavros
- **ROS Noetic Tutorials:** http://wiki.ros.org/ROS/Tutorials
- **WSL Documentation:** https://learn.microsoft.com/en-us/windows/wsl/

---

## Summary

You now have a complete development environment for UAV simulation:

✅ PX4 Autopilot v1.14.3 in WSL2  
✅ Gazebo Classic for 3D simulation  
✅ QGroundControl for ground station monitoring  
✅ ROS1 Noetic with MAVROS for programmatic control  

**Next Steps:**
- Explore different vehicle types
- Learn PX4 flight modes
- Write ROS nodes to control the drone
- Plan autonomous missions
- Test failsafe scenarios

Happy flying! 🚁

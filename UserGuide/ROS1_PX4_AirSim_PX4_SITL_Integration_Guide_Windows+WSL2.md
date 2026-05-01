# AirSim + PX4 SITL Integration Guide (Windows + WSL2)

## Complete Setup Guide for Testing AirLib Before Real Drone Deployment

---

## Overview

This guide shows you how to:
1. Compile AirLib v1.6.0 on Ubuntu (WSL2)
2. Run AirSim simulator on Windows
3. Connect AirSim to PX4 SITL running in WSL2
4. Test the complete workflow before deploying to Raspberry Pi + Pixhawk

**Purpose:** Verify AirLib works with PX4 in simulation before spending time on cross-compilation and hardware deployment.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Part 1: Compile AirLib in WSL2](#part-1-compile-airlib-in-wsl2)
3. [Part 2: Download AirSim Precompiled Binary (Windows)](#part-2-download-airsim-precompiled-binary-windows)
4. [Part 3: Network Configuration (WSL2 + Windows)](#part-3-network-configuration-wsl2--windows)
5. [Part 4: Configure AirSim Settings](#part-4-configure-airsim-settings)
6. [Part 5: Start PX4 SITL](#part-5-start-px4-sitl)
7. [Part 6: Run AirSim Simulator](#part-6-run-airsim-simulator)
8. [Part 7: Test with Python API](#part-7-test-with-python-api)
9. [Troubleshooting](#troubleshooting)
10. [Next Steps](#next-steps)

---

## Prerequisites

### Hardware Requirements
- Windows 10/11 with WSL2 enabled
- 16GB RAM minimum (8GB for Windows, 8GB for WSL2)
- 50GB free disk space
- Dedicated GPU recommended (for AirSim graphics)

### Software Requirements
- **Windows:** Latest Windows 10/11 with WSL2
- **WSL2:** Ubuntu 20.04 or 22.04
- **PX4:** v1.14.3 already installed in WSL2
- **Python:** 3.8+ (Windows or WSL2)

### Already Completed (from previous sessions)
- ✅ PX4 Autopilot v1.14.3 installed in WSL2
- ✅ Basic PX4 SITL tested with Gazebo

---

## Part 1: Compile AirLib in WSL2

### Step 1.1: Install Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install build essentials
sudo apt install -y \
    build-essential \
    cmake \
    git \
    gcc \
    g++ \
    clang

# Install required libraries
sudo apt install -y \
    libboost-all-dev \
    libeigen3-dev \
    libssl-dev \
    uuid-dev
```

---

### Step 1.2: Clone AirSim Repository

```bash
cd ~
git clone https://github.com/microsoft/AirSim.git
cd AirSim

# Use v1.6.0-linux (most stable for Linux)
git checkout v1.6.0-linux

# Update submodules
git submodule update --init --recursive
```

---

### Step 1.3: Build AirLib

```bash
cd ~/AirSim

# Run setup script
./setup.sh

# Build AirLib (this takes 5-10 minutes)
./build.sh
```

**Expected output:**
```
Building AirLib...
Building MavLinkCom...
Building RPC library...
Build completed successfully!
```

---

### Step 1.4: Verify Build

```bash
# Check if binaries exist
ls ~/AirSim/build_release/output/bin/

# Should show:
# MavLinkTest
# DroneServer
# HelloDrone
# DroneShell
```

**If build succeeds, AirLib is compiled successfully!**

---

## Part 2: Download AirSim Precompiled Binary (Windows)

### Step 2.1: Download AirSim Environment

**Option 1: Blocks (Recommended - Small and Simple)**

1. Go to: https://github.com/microsoft/AirSim/releases/tag/v1.6.0-windows
2. Download **Blocks.zip** (~140MB)
3. Extract to `C:\AirSim\Blocks\`

**Option 2: MSBuild2018 (Soccer Field)**

1. Go to: https://github.com/microsoft/AirSim/releases/tag/v1.6.0-windows
2. Download **MSBuild2018.zip** (~500MB)
3. Extract to `C:\AirSim\MSBuild2018\`

**Option 3: LandscapeMountains (More Complex Terrain)**

1. Download **LandscapeMountains.zip**
2. Extract to `C:\AirSim\LandscapeMountains\`

---

### Step 2.2: Verify Extraction

Navigate to the extracted folder. You should see:

```
C:\AirSim\Blocks\
├── Blocks.exe          (Main executable)
├── Engine/
├── Blocks/
└── ... (other Unreal Engine files)
```

---

## Part 3: Network Configuration (WSL2 + Windows)

### Step 3.1: Find Your Windows IP Address

Open **PowerShell** on Windows:

```powershell
ipconfig
```

Look for **"Ethernet adapter vEthernet (WSL)"** or **"vEthernet (WSL (Hyper-V firewall))"**

Example output:
```
Ethernet adapter vEthernet (WSL (Hyper-V firewall)):

   Connection-specific DNS Suffix  . :
   Link-local IPv6 Address . . . . . : fe80::xxxx:xxxx:xxxx:xxxx%xx
   IPv4 Address. . . . . . . . . . . : 172.28.0.1     <-- THIS IS YOUR WINDOWS IP
   Subnet Mask . . . . . . . . . . . : 255.255.240.0
   Default Gateway . . . . . . . . . :
```

**Write down this IP address** (e.g., `172.28.0.1`)

---

### Step 3.2: Find Your WSL2 IP Address (Optional)

In **WSL2 terminal**:

```bash
ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1
```

Example output:
```
172.28.15.216     <-- This is your WSL2 IP
```

**Note:** We need the **Windows IP** for AirSim configuration, not the WSL2 IP.

---

### Step 3.3: Open Windows Firewall Ports

Open **PowerShell as Administrator**:

```powershell
# Allow TCP port 4560 (AirSim-PX4 connection)
New-NetFirewallRule -DisplayName "AirSim PX4 TCP" -Direction Inbound -Protocol TCP -LocalPort 4560 -Action Allow

# Allow UDP port 14540 (MAVLink Control)
New-NetFirewallRule -DisplayName "AirSim MAVLink Control" -Direction Inbound -Protocol UDP -LocalPort 14540 -Action Allow

# Allow UDP port 14580 (MAVLink Remote)
New-NetFirewallRule -DisplayName "AirSim MAVLink Remote" -Direction Inbound -Protocol UDP -LocalPort 14580 -Action Allow
```

**Verify firewall rules created:**

```powershell
Get-NetFirewallRule -DisplayName "AirSim*"
```

---

## Part 4: Configure AirSim Settings

### Step 4.1: Create Settings Directory

On **Windows**, create the directory:

```
C:\Users\YourUsername\Documents\AirSim\
```

**Replace `YourUsername`** with your actual Windows username.

---

### Step 4.2: Create settings.json

Create a file: `C:\Users\YourUsername\Documents\AirSim\settings.json`

**Copy this content:**

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

**IMPORTANT:** Replace `"LocalHostIp": "172.28.0.1"` with **your Windows IP** from Step 3.1.

---

### Step 4.3: Understand the Settings

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SimMode` | `Multirotor` | Drone simulation mode |
| `ClockType` | `SteppableClock` | Synchronized time between AirSim and PX4 |
| `UseTcp` | `true` | Use TCP for simulator connection |
| `TcpPort` | `4560` | PX4 expects simulator on this port |
| `ControlIp` | `remote` | Auto-detect WSL2 IP for MAVLink |
| `ControlPortLocal` | `14540` | Local MAVLink port |
| `ControlPortRemote` | `14580` | Remote MAVLink port |
| `LocalHostIp` | `172.28.0.1` | **Your Windows IP** (CHANGE THIS!) |
| `LockStep` | `true` | Synchronize PX4 and AirSim time steps |

---

## Part 5: Start PX4 SITL

### Step 5.1: Set Environment Variable (WSL2)

In **WSL2 terminal**:

```bash
# Set PX4 to connect to Windows AirSim
export PX4_SIM_HOST_ADDR=172.28.0.1  # Replace with YOUR Windows IP

# Verify it's set
echo $PX4_SIM_HOST_ADDR
```

**CRITICAL:** This IP must match `LocalHostIp` in your settings.json!

---

### Step 5.2: Start PX4 SITL

```bash
cd ~/PX4-Autopilot

# Start PX4 SITL (headless, no Gazebo)
make px4_sitl none_iris
```

**Expected output:**

```
______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [dataman] data manager file './dataman' size is 7866640 bytes
INFO  [init] shell id: 1996293728
INFO  [tone_alarm] home set
INFO  [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
```

**PX4 is now waiting for AirSim to connect.**

---

## Part 6: Run AirSim Simulator

### Step 6.1: Launch AirSim (Windows)

**Option 1: Double-click the executable**

Navigate to `C:\AirSim\Blocks\` and double-click `Blocks.exe`

**Option 2: Run from Command Prompt with options**

Open **Command Prompt** (Windows):

```cmd
cd C:\AirSim\Blocks
Blocks.exe -ResX=1280 -ResY=720 -windowed
```

**Command-line options:**
- `-ResX=1280 -ResY=720` : Set resolution
- `-windowed` : Run in windowed mode (not fullscreen)
- `-quality=Low` : Reduce graphics quality for better performance

---

### Step 6.2: Verify Connection

**In AirSim window (Windows), you should see:**

```
Loaded settings from C:\Users\lukas\Documents\AirSim\settings.json
Asset database ready!
Press F1 to see help
Camera: ExternalCamera
Collision Count: 0
Disconnecting mavlink vehicle
Waiting for mavlink vehicle...
Waiting for TCP connection on port 4560, local IP 172.28.0.1
Simulator connected on TCP port 4560
Connected to PX4 Ground Control
```

**In PX4 terminal (WSL2), you should see:**

```
INFO  [simulator_mavlink] Simulator connected on TCP port 4560
INFO  [mavlink] partner IP: 172.28.0.1
INFO  [ecl/EKF] EKF GPS checks passed (WGS-84 origin set)
INFO  [ecl/EKF] EKF commencing GPS fusion
```

**If you see these messages, the connection is successful!** ✅

---

### Step 6.3: AirSim Controls

Once AirSim is running:

| Key | Action |
|-----|--------|
| **F1** | Show help menu |
| **F10** | Toggle recording |
| **Backspace** | Reset drone to start position |
| **Arrow Keys** | Manual control (if RC enabled) |
| **PgUp/PgDn** | Increase/decrease altitude (manual mode) |
| **;** | Toggle camera view |
| **ESC** | Exit AirSim |

---

## Part 7: Test with Python API

### Step 7.1: Install AirSim Python Package

**Option A: Install on Windows**

Open **Command Prompt** or **PowerShell**:

```cmd
pip install airsim
```

**Option B: Install on WSL2**

```bash
cd ~/AirSim/PythonClient
pip3 install --user -e .
```

---

### Step 7.2: Create Test Script

Create `test_airsim_px4.py`:

```python
#!/usr/bin/env python3
import airsim
import time

print("=" * 50)
print("AirSim + PX4 SITL Connection Test")
print("=" * 50)

# Connect to AirSim
print("\n[1/8] Connecting to AirSim...")
client = airsim.MultirotorClient(ip="172.23.0.1") # This is referring to windows IP address "IPv4 Address"
#client = airsim.MultirotorClient()
client.confirmConnection()
print("✓ Connected to AirSim")

# Enable API control
print("\n[2/8] Enabling API control...")
client.enableApiControl(True)
print("✓ API control enabled")

# Arm the drone
print("\n[3/8] Arming drone...")
client.armDisarm(True)
time.sleep(1)
print("✓ Drone armed")

# Takeoff
print("\n[4/8] Taking off...")
client.takeoffAsync().join()
print("✓ Takeoff complete")

# Hover and get telemetry
print("\n[5/8] Hovering for 5 seconds...")
for i in range(5):
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    print(f"  T+{i+1}s - Position: X={pos.x_val:.2f}m, Y={pos.y_val:.2f}m, Z={pos.z_val:.2f}m")
    time.sleep(1)

print("✓ Hover complete")

# Land
print("\n[6/8] Landing...")
client.landAsync().join()
print("✓ Landing complete")

# Disarm
print("\n[7/8] Disarming...")
client.armDisarm(False)
print("✓ Drone disarmed")

# Disable API control
print("\n[8/8] Disabling API control...")
client.enableApiControl(False)
print("✓ API control disabled")

print("\n" + "=" * 50)
print("✅ TEST SUCCESSFUL!")
print("AirLib is working with PX4 SITL!")
print("=" * 50)
```

---

### Step 7.3: Run Test Script

**On Windows:**

```cmd
python test_airsim_px4.py
```

**On WSL2:**

```bash
python3 test_airsim_px4.py
```

---

### Step 7.4: Expected Output

```
==================================================
AirSim + PX4 SITL Connection Test
==================================================

[1/8] Connecting to AirSim...
✓ Connected to AirSim

[2/8] Enabling API control...
✓ API control enabled

[3/8] Arming drone...
✓ Drone armed

[4/8] Taking off...
✓ Takeoff complete

[5/8] Hovering for 5 seconds...
  T+1s - Position: X=0.12m, Y=-0.05m, Z=-2.48m
  T+2s - Position: X=0.11m, Y=-0.04m, Z=-2.50m
  T+3s - Position: X=0.10m, Y=-0.06m, Z=-2.51m
  T+4s - Position: X=0.12m, Y=-0.05m, Z=-2.49m
  T+5s - Position: X=0.11m, Y=-0.04m, Z=-2.50m
✓ Hover complete

[6/8] Landing...
✓ Landing complete

[7/8] Disarming...
✓ Drone disarmed

[8/8] Disabling API control...
✓ API control disabled

==================================================
✅ TEST SUCCESSFUL!
AirLib is working with PX4 SITL!
==================================================
```

**You should also see the drone in AirSim window taking off, hovering, and landing!**

---

## Troubleshooting

### Issue 1: "Waiting for simulator" - Connection Never Established

**Symptom:**
```
INFO  [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
```
Never changes to "Simulator connected"

**Solutions:**

1. **Check Windows IP is correct:**
   ```powershell
   ipconfig
   ```
   Verify IP in settings.json matches vEthernet (WSL) adapter.

2. **Check PX4 environment variable:**
   ```bash
   echo $PX4_SIM_HOST_ADDR
   ```
   Should match Windows IP.

3. **Check firewall:**
   Temporarily disable Windows Firewall to test, then re-enable with proper rules.

4. **Restart both:**
   - Close AirSim
   - Stop PX4 (Ctrl+C)
   - Start PX4 first
   - Then start AirSim

---

### Issue 2: "TcpClientPort socket bind failed with error: 10049"

**Symptom:**
```
Accepting TCP socket failed, is another instance running?
TcpClientPort socket bind failed with error: 10049
```

**Cause:** `LocalHostIp` in settings.json is set to WSL IP instead of Windows IP.

**Solution:**

Change settings.json:

```json
"LocalHostIp": "172.28.0.1"  // Windows IP, not WSL IP!
```

**OR use:**

```json
"LocalHostIp": "0.0.0.0"  // Listen on all interfaces
```

---

### Issue 3: Python Script "Connection refused"

**Symptom:**
```python
ConnectionRefusedError: [Errno 111] Connection refused
```

**Solutions:**

1. **Verify AirSim is running** (see drone in window)

2. **Check if DroneServer is needed:**
   - For Windows → AirSim connection: No DroneServer needed
   - For WSL2 → AirSim connection: May need DroneServer

3. **Try connecting from same machine as AirSim:**
   - If AirSim on Windows, run Python on Windows
   - If running from WSL2, connect to Windows IP

---

### Issue 4: Drone Doesn't Respond to Commands

**Symptom:** Python script connects but drone doesn't move.

**Check in PX4 console:**

```bash
# In PX4 pxh> console
commander status
```

Should show armed state and flight mode.

**Check GPS lock:**

```bash
listener vehicle_gps_position -n 1
```

GPS fix should be `3` or higher.

**Wait for home position:**

PX4 console should show:
```
INFO  [commander] home: 47.6414680, -122.1401672, 119.99
```

---

### Issue 5: AirSim Crashes on Startup

**Symptom:** AirSim window opens then immediately closes.

**Solutions:**

1. **Check graphics drivers** (update GPU drivers)

2. **Reduce graphics quality:**
   ```cmd
   Blocks.exe -quality=Low
   ```

3. **Run in windowed mode:**
   ```cmd
   Blocks.exe -windowed -ResX=1024 -ResY=768
   ```

4. **Check settings.json is valid JSON:**
   Use https://jsonlint.com/ to validate

---

### Issue 6: WSL IP Changes After Reboot

**Symptom:** Connection works, then stops working after reboot.

**Cause:** WSL2 IP addresses are dynamic and change on reboot.

**Solution 1: Check and update IP after each reboot**

```bash
# In WSL2
ip addr show eth0 | grep inet

# Update settings.json with new Windows IP
# Restart PX4 with new export PX4_SIM_HOST_ADDR=<new_ip>
```

**Solution 2: Use 0.0.0.0 in settings.json**

```json
"LocalHostIp": "0.0.0.0"
```

This listens on all interfaces.

**Solution 3: Enable WSL2 Mirrored Networking (Windows 11 22H2+)**

Create `C:\Users\YourUsername\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Restart WSL:
```powershell
wsl --shutdown
```

Now you can use `127.0.0.1` (localhost) instead of dynamic IPs.

---

## Next Steps

### What We've Accomplished

✅ **AirLib v1.6.0 compiled** in WSL2  
✅ **AirSim simulator running** on Windows  
✅ **PX4 SITL connected** to AirSim via WSL2 network bridge  
✅ **Python API tested** - arm, takeoff, hover, land  
✅ **Complete simulation workflow** verified

---

### Ready for Real Hardware Deployment

Now that simulation works, you can:

1. **Cross-compile AirLib for Raspberry Pi** (ARM64)
2. **Deploy binaries to Raspberry Pi**
3. **Connect Raspberry Pi to Pixhawk via USB**
4. **Run the same Python scripts on real drone**

---

### Cross-Compilation Guide

**Next guide will cover:**

1. Install ARM cross-compiler on WSL2
2. Cross-compile AirLib for ARM64 (Raspberry Pi 4)
3. Deploy to Raspberry Pi
4. Configure Pixhawk PX4 parameters
5. Test connection with MavLinkTest
6. Run DroneServer on Raspberry Pi
7. Execute flight scripts on real hardware

---

## Summary of Key Files and Locations

### Windows

| File/Directory | Purpose |
|----------------|---------|
| `C:\AirSim\Blocks\` | AirSim simulator executable |
| `C:\Users\YourUsername\Documents\AirSim\settings.json` | AirSim configuration |

### WSL2

| File/Directory | Purpose |
|----------------|---------|
| `~/PX4-Autopilot/` | PX4 SITL firmware |
| `~/AirSim/` | AirSim source code |
| `~/AirSim/build_release/output/bin/` | Compiled AirLib binaries |
| `~/test_airsim_px4.py` | Python test script |

---

## Important Network Information

**Record these for your setup:**

- **Windows IP (vEthernet WSL):** `172.28.0.1` ← **Update this with YOUR IP**
- **WSL2 IP:** `172.28.15.216` ← **This changes on reboot**
- **AirSim TCP Port:** `4560`
- **MAVLink Control Port:** `14540`
- **MAVLink Remote Port:** `14580`
- **DroneServer API Port:** `41451`

---

## Reference Commands

### Start Complete Workflow

**Terminal 1 (WSL2) - Start PX4:**

```bash
export PX4_SIM_HOST_ADDR=172.28.0.1  # Your Windows IP
cd ~/PX4-Autopilot
make px4_sitl none_iris
```

**Windows - Start AirSim:**

```cmd
cd C:\AirSim\Blocks
Blocks.exe -windowed
```

**Terminal 2 (Windows or WSL2) - Run Python Test:**

```bash
python test_airsim_px4.py
```

---

## Frequently Asked Questions

### Q: Can I run AirSim on WSL2 instead of Windows?

**A:** Theoretically yes with X server, but graphics performance is poor. Windows is recommended for AirSim.

---

### Q: Do I need DroneServer for simulation?

**A:** No. Python client connects directly to AirSim. DroneServer is only needed for real drones.

---

### Q: Can I use QGroundControl with this setup?

**A:** Yes! QGC will automatically connect to PX4 SITL on UDP port 14550.

---

### Q: How do I save and replay flights?

**A:** Use MavLinkTest with `-logdir` flag to record flights as `.mavlink` files.

---

### Q: My firewall keeps blocking. How to fix permanently?

**A:** Add the PowerShell firewall rules from Step 3.3 permanently. They persist across reboots.

---

## Additional Resources

- **AirSim Documentation:** https://microsoft.github.io/AirSim/
- **PX4 User Guide:** https://docs.px4.io/
- **PX4 SITL with AirSim:** https://microsoft.github.io/AirSim/px4_sitl/
- **WSL2 Networking:** https://docs.microsoft.com/en-us/windows/wsl/networking

---

## Changelog

| Date | Version | Changes |
|------|---------|---------|
| 2026-05-01 | 1.0 | Initial guide - AirSim Windows + PX4 SITL WSL2 integration |

---

**Congratulations!** 🎉

You've successfully set up and tested the complete AirSim + PX4 SITL workflow. You're now ready to proceed to cross-compilation and real drone deployment.

**Next Guide:** Cross-Compiling AirLib for Raspberry Pi + Real Pixhawk Deployment

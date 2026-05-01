# PX4 Commander Commands Reference Guide

## Overview

This guide provides a comprehensive reference for PX4 commander commands and system status checking. These commands are used in the PX4 console (nsh shell) for controlling the drone, checking status, and debugging.

**Accessing the PX4 Console:**
- **SITL (Simulation):** Commands appear directly in the terminal where you launched `make px4_sitl gazebo-classic_iris`
- **Hardware:** Connect via QGroundControl or serial console (USB/UART)

---

## Table of Contents

1. [Commander Basic Commands](#commander-basic-commands)
2. [Flight Mode Commands](#flight-mode-commands)
3. [Parameter Commands](#parameter-commands)
4. [Status Checking Commands](#status-checking-commands)
5. [Listener Commands (uORB Topics)](#listener-commands-uorb-topics)
6. [Calibration Commands](#calibration-commands)
7. [System Commands](#system-commands)
8. [MAVLink Commands](#mavlink-commands)
9. [Debugging Commands](#debugging-commands)
10. [Quick Reference](#quick-reference)

---

## Commander Basic Commands

### Arming and Disarming

```bash
# Arm the vehicle (enable motors)
commander arm

# Disarm the vehicle (disable motors)
commander disarm

# Force arm (bypass preflight checks - DANGEROUS)
commander arm -f

# Force disarm (even in flight - DANGEROUS)
commander disarm -f
```

**Safety Note:** Never use force arm/disarm unless you know exactly what you're doing.

---

### Takeoff and Landing

```bash
# Takeoff to default altitude (2.5m)
commander takeoff

# Land at current position
commander land
```

**Note:** `commander takeoff` does not accept altitude parameters. To set altitude, use parameters or QGroundControl.

---

### Set Takeoff Altitude

```bash
# Set default takeoff altitude (in meters)
param set MIS_TAKEOFF_ALT 10

# Then takeoff
commander takeoff
```

---

## Flight Mode Commands

### Switch Flight Modes

```bash
# Manual mode (full manual control)
commander mode manual

# Altitude control mode (manual horizontal, altitude hold)
commander mode altctl

# Position control mode (GPS position hold)
commander mode posctl

# Auto mode (mission/RTL/etc)
commander mode auto

# Offboard mode (external control via MAVROS/MAVLink)
commander mode offboard

# Return to Launch (RTL)
commander mode rtl

# Hold position (loiter)
commander mode loiter

# Acro mode (rate control, no stabilization)
commander mode acro

# Stabilized mode (attitude stabilization)
commander mode stabilized
```

---

## Parameter Commands

### View Parameters

```bash
# Show specific parameter value
param show MIS_TAKEOFF_ALT

# Show all parameters matching pattern (wildcard)
param show SYS*

# Show all parameters (VERY LONG LIST)
param show
```

---

### Set Parameters

```bash
# Set a parameter value
param set PARAM_NAME value

# Examples:
param set MIS_TAKEOFF_ALT 10        # Takeoff altitude 10m
param set COM_DISARM_LAND 5         # Auto-disarm 5 seconds after landing
param set RTL_RETURN_ALT 30         # RTL altitude 30m
param set SYS_AUTOSTART 4001        # Set airframe type
```

---

### Save and Reset Parameters

```bash
# Save current parameters (automatic after param set)
param save

# Reset all parameters to defaults
param reset

# Reset specific parameter
param reset PARAM_NAME
```

---

## Status Checking Commands

### Commander Status

```bash
# Show commander state and armed status
commander status
```

**Output includes:**
- Arming state (ARMED/DISARMED)
- Flight mode (MANUAL/AUTO/POSCTL/etc)
- Navigation state
- Failsafe status
- Home position status

---

### System Status

```bash
# Show overall system health
commander check

# Preflight check
commander preflight_check
```

---

### Battery Status

```bash
# View battery information
listener battery_status -n 1
```

**Output:**
- Voltage (V)
- Current (A)
- Remaining capacity (%)
- Cells info

---

### GPS Status

```bash
# Check GPS fix and satellite count
listener vehicle_gps_position -n 1
```

**Output:**
- Fix type (0=none, 3=3D, 4=DGPS, 5=RTK)
- Number of satellites
- Horizontal/vertical accuracy
- Latitude, longitude, altitude

---

### Position and Attitude

```bash
# Local position (NED frame)
listener vehicle_local_position -n 1

# Global position (GPS)
listener vehicle_global_position -n 1

# Attitude (roll, pitch, yaw)
listener vehicle_attitude -n 1
```

**NED Frame:**
- `x` = North (m)
- `y` = East (m)
- `z` = Down (m, **negative = altitude**)

---

### Sensor Status

```bash
# IMU data (accelerometer + gyroscope)
listener sensor_combined -n 1

# Magnetometer (compass)
listener sensor_mag -n 1

# Barometer (pressure altitude)
listener sensor_baro -n 1
```

---

## Listener Commands (uORB Topics)

The `listener` command monitors internal PX4 messages (uORB topics).

### Basic Syntax

```bash
listener <topic_name> [options]

# Options:
#   -n <num>  Number of messages to print (default: 1)
#   -r <rate> Subscription rate in Hz
#   -i <inst> Topic instance (for multi-instance topics)
```

---

### Essential Topics

```bash
# Vehicle state
listener vehicle_status -n 1

# Connection state (MAVROS/GCS)
listener vehicle_command

# Arming/safety state
listener actuator_armed -n 1

# RC input (radio control)
listener input_rc -n 1

# Setpoints (commanded position/attitude)
listener vehicle_local_position_setpoint -n 1

# Actuator outputs (motor/servo commands)
listener actuator_outputs -n 1

# Failsafe status
listener failsafe_flags -n 1

# EKF (estimator) status
listener estimator_status -n 1
```

---

### Continuous Monitoring

```bash
# Monitor topic continuously (Ctrl+C to stop)
listener vehicle_local_position

# Monitor at specific rate (10 Hz)
listener vehicle_local_position -r 10
```

---

### List All Topics

```bash
# Show all available uORB topics
listener
```

---

## Calibration Commands

### Accelerometer Calibration

```bash
commander calibrate accel
```

Follow on-screen instructions to rotate vehicle to each orientation.

---

### Gyroscope Calibration

```bash
commander calibrate gyro
```

Keep vehicle stationary during calibration.

---

### Magnetometer (Compass) Calibration

```bash
commander calibrate mag
```

Rotate vehicle through all orientations as prompted.

---

### Level Horizon Calibration

```bash
commander calibrate level
```

Place vehicle on level surface.

---

### Airspeed Calibration (Fixed-wing)

```bash
commander calibrate airspeed
```

---

### All Sensors Calibration

```bash
commander calibrate
```

Runs all calibration procedures sequentially.

---

## System Commands

### Version Information

```bash
# Show PX4 version
ver all

# Show hardware info
ver hw

# Show git version
ver git

# Show build date
ver bdate
```

---

### Reboot and Shutdown

```bash
# Reboot PX4
reboot

# Shutdown PX4 (SITL only)
shutdown
```

---

### List Running Modules

```bash
# Show all running modules
top

# Show specific module status
<module_name> status

# Examples:
commander status
ekf2 status
mc_pos_control status
```

---

### Start/Stop Modules

```bash
# Start a module
<module_name> start

# Stop a module
<module_name> stop

# Examples:
logger start
mavlink stop
```

---

## MAVLink Commands

### MAVLink Status

```bash
# Show all MAVLink instances and connections
mavlink status
```

**Output includes:**
- Instance number
- Mode (Normal/Onboard/etc)
- Transport protocol (UDP/Serial)
- Port numbers
- Data rates
- Partner IP addresses

---

### Start MAVLink Instance

```bash
# Start MAVLink on specific port
mavlink start -d /dev/ttyS1 -b 57600
```

---

## Debugging Commands

### Test Motor/Actuator

```bash
# Test specific motor (REMOVE PROPS FIRST!)
motor_test test -m 1 -p 10

# Options:
#   -m <motor>    Motor number (1-8)
#   -p <power>    Power percentage (0-100)
#   -t <time>     Duration in seconds
```

**SAFETY WARNING:** Always remove propellers before testing motors!

---

### Preflight Check Details

```bash
# Run detailed preflight check
commander check
```

Shows all preflight check results:
- Sensors initialized
- Home position set
- GPS lock
- Battery OK
- RC connected (if required)
- Arming authorization

---

### Simulate Failsafes

```bash
# Simulate GPS failure
failure gps off

# Restore GPS
failure gps ok

# Simulate battery failure
failure battery off
```

---

### Log Messages

```bash
# Show recent log messages
dmesg

# Monitor system logs in real-time
listener log_message
```

---

### Performance Monitoring

```bash
# CPU and task load
top

# Free memory
free

# SD card performance test
sd_bench
```

---

## Quick Reference

### Flight Workflow

```bash
# 1. Check system status
commander status

# 2. Check GPS lock
listener vehicle_gps_position -n 1

# 3. Set takeoff altitude (optional)
param set MIS_TAKEOFF_ALT 10

# 4. Arm
commander arm

# 5. Takeoff
commander takeoff

# 6. Switch to position control (if needed)
commander mode posctl

# 7. Land
commander land

# 8. Disarm (automatic after landing)
commander disarm
```

---

### Essential Status Checks

```bash
# Quick health check
commander status                        # Armed state, flight mode
listener battery_status -n 1           # Battery voltage, remaining %
listener vehicle_gps_position -n 1     # GPS fix, satellites
listener vehicle_local_position -n 1   # Current altitude (z is negative)
mavlink status                          # GCS connection status
```

---

### Common Parameters

| Parameter | Description | Default | Units |
|-----------|-------------|---------|-------|
| `MIS_TAKEOFF_ALT` | Takeoff altitude | 2.5 | meters |
| `RTL_RETURN_ALT` | RTL return altitude | 30 | meters |
| `COM_DISARM_LAND` | Auto-disarm after landing | 2 | seconds |
| `COM_RC_LOSS_T` | RC loss timeout | 0.5 | seconds |
| `COM_RC_IN_MODE` | RC input mode | 0 (stick) | - |
| `NAV_ACC_RAD` | Acceptance radius (waypoint) | 5 | meters |
| `MPC_Z_VEL_MAX_UP` | Max vertical speed (up) | 3 | m/s |
| `MPC_Z_VEL_MAX_DN` | Max vertical speed (down) | 1.5 | m/s |

---

### Keyboard Shortcuts (PX4 Console)

| Key | Action |
|-----|--------|
| `Ctrl+C` | Stop current command |
| `Ctrl+D` | Exit console (SITL) |
| `Tab` | Auto-complete command |
| `↑ / ↓` | Command history |
| `Ctrl+L` | Clear screen |

---

## Troubleshooting

### "Commander: arming denied"

Check arming requirements:

```bash
commander check
listener vehicle_status -n 1
```

Common causes:
- GPS not locked (need 8+ satellites)
- Home position not set
- Preflight check failed
- Battery too low
- RC not connected (if required)

**Fix:**

```bash
# Check GPS
listener vehicle_gps_position -n 1

# Check home position
listener home_position -n 1

# Force arm (if safe)
commander arm -f
```

---

### "Mode transition denied"

Check current mode and prerequisites:

```bash
commander status
listener vehicle_status -n 1
```

Some modes require:
- **POSCTL:** GPS lock + home position
- **OFFBOARD:** Active offboard commands
- **AUTO:** Uploaded mission

---

### Motors Not Spinning After Arm

Check actuator outputs:

```bash
listener actuator_outputs -n 1
```

If all zeros, check:

```bash
# Ensure not in HITL mode
param show SYS_HITL
# Should be 0 for normal operation

# Check if motors are mapped correctly
param show PWM_MAIN*
```

---

### GPS Not Getting Fix

```bash
# Check GPS status
listener vehicle_gps_position -n 1

# Should show:
#   fix_type: 3 (3D fix) or higher
#   satellites_used: 8+
```

**In Simulation:** GPS fix should happen within 5-10 seconds.

**On Hardware:** May take 1-5 minutes outdoors with clear sky view.

---

## Advanced Commands

### Set Home Position Manually

```bash
# Set home to current position
commander set_home

# Set home to specific GPS coordinates
# (This typically requires MAVLink command)
```

---

### Change Airframe

```bash
# Set airframe type
param set SYS_AUTOSTART 4001

# Enable auto-configuration
param set SYS_AUTOCONFIG 1

# Reboot to apply
reboot
```

Common airframe IDs:
- `4001` - Generic Quadcopter (X)
- `4011` - DJI Flame Wheel F450
- `13000` - Generic VTOL Standard
- `2100` - Generic Plane

---

### Flight Log Management

```bash
# Start logging
logger start

# Stop logging
logger stop

# Show logger status
logger status
```

Logs are saved to SD card (or simulation directory) as `.ulg` files.

---

## Summary

**Most Common Commands:**

```bash
# Status
commander status
mavlink status

# Flight
commander arm
commander takeoff
commander land

# Monitoring
listener vehicle_local_position -n 1
listener vehicle_gps_position -n 1
listener battery_status -n 1

# Parameters
param show MIS_TAKEOFF_ALT
param set MIS_TAKEOFF_ALT 10

# Help
help                    # List all commands
<command> help          # Help for specific command
```

---

## Additional Resources

- **PX4 User Guide:** https://docs.px4.io/
- **Modules Reference:** https://docs.px4.io/main/en/modules/modules_main.html
- **uORB Topics:** https://docs.px4.io/main/en/msg_docs/
- **Parameters Reference:** https://docs.px4.io/main/en/advanced_config/parameter_reference.html

---

**Pro Tip:** Use `help` command in PX4 console to see all available commands. Most commands support `<command> help` for detailed usage.

Happy flying! 🚁

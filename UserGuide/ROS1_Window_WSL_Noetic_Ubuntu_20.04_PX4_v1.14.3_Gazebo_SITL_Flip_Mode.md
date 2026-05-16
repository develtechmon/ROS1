# ROS1 | WSL2 | Noetic | Ubuntu 20.04 | PX4 v1.14.3 | Gazebo SITL Flip Mode

> **Purpose:** Complete reference for running autonomous drone flip and recovery using PX4 v1.14.3 SITL with Gazebo Classic (iris model), MAVROS, and ROS Noetic on WSL2.
>
> **Companion document:** See `ROS1_Windows_WSL_Noetic_Ubuntu_20.04_PX4_v1.14.3_SITL_Flip_Mode.md` for the AirSim version. This document covers differences specific to Gazebo and explains why the tuning values differ.

---

## Table of Contents

1. [AirSim vs Gazebo — Which Reflects Real-World Physics?](#1-airsim-vs-gazebo--which-reflects-real-world-physics)
2. [Why the Same Script Behaves Differently](#2-why-the-same-script-behaves-differently)
3. [Environment Setup](#3-environment-setup)
4. [Startup Sequence](#4-startup-sequence)
5. [PX4 Parameters for Gazebo](#5-px4-parameters-for-gazebo)
6. [Flip Script — Gazebo Tuned](#6-flip-script--gazebo-tuned)
7. [Tuning Reference](#7-tuning-reference)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. AirSim vs Gazebo — Which Reflects Real-World Physics?

This is a critical question for your PhD research and real drone deployment.

### Short Answer

**Neither is perfectly realistic — but Gazebo is closer to real-world flight dynamics**, which is why PX4 officially recommends Gazebo for SITL development.

### Detailed Comparison

| Aspect | AirSim | Gazebo | Real Drone |
|--------|--------|--------|------------|
| **Physics engine** | FastPhysics (Microsoft, simplified) | ODE / Bullet (robotics-standard) | Real world |
| **Motor model** | Simplified, near-instant response | Full inertia + rotor drag model | Real inertia, lag |
| **Hover thrust** | ~35% (lighter feel) | ~50% (heavier, more realistic) | Typically 40–60% |
| **Air drag** | Minimal | Modeled (rotor drag coefficient) | Significant |
| **Altitude drop during flip** | Small | Large (realistic) | Significant |
| **Gravity effect** | Lighter, forgiving | Full 9.81 m/s² effect | Full 9.81 m/s² |
| **Sim-to-real gap** | Larger | Smaller | — |
| **Visuals** | Photorealistic (Unreal Engine) | Basic but functional | — |
| **PX4 recommendation** | Supported | **Officially recommended** | — |
| **Community** | Smaller (archived 2022) | Large, actively maintained | — |

### What This Means for Your Research

**AirSim masked a real problem.** When the AirSim flip script used `thrust=0.2` during the rotation, the drone barely dropped because AirSim's physics are forgiving. The same script in Gazebo caused the drone to crash because Gazebo correctly models that `thrust=0.2 < hover_thrust=0.50` means the drone is in freefall.

**Gazebo exposed the truth:** On a real drone, commanding 20% thrust during a flip while hover requires 50% means your motors are providing less than half the force needed to fight gravity. The drone will drop significantly. Your Gazebo tuning (`thrust=0.55` during flip) is therefore closer to what you will need on real hardware.

**Analogy:** AirSim is like practicing skateboard tricks on a foam mat — forgiving if you fall. Gazebo is practicing on concrete — you have to get it right or it hurts. Real drone is the concrete.

### Bottom Line for Your PhD

| Phase | Use This |
|-------|----------|
| **PPO training (fast iterations)** | AirSim — faster reset, easier disturbance injection |
| **Algorithm validation** | Gazebo — more realistic physics, closer to real behavior |
| **Pre-deployment testing** | Gazebo HITL with real Pixhawk |
| **Real drone** | Real hardware |

**Expect the real drone to behave more like Gazebo than AirSim.** Your Gazebo-tuned parameters are a better starting point for real flight.

---

## 2. Why the Same Script Behaves Differently

### Root Cause: `MPC_THR_HOVER` Difference

The single most important difference is the hover thrust value:

| Simulator | `MPC_THR_HOVER` | What `thrust=0.2` means |
|-----------|-----------------|------------------------|
| AirSim | ~0.35 | Slightly below hover — small drop |
| **Gazebo iris** | **0.50** | **Well below hover — freefall** |
| Real drone | ~0.40–0.60 | Depends on weight/motors |

During the flip, the script commands `thrust=0.2` for ~0.4 seconds. In Gazebo:

```
Drone weight force:  mass × 9.81 m/s²    (always pulling down)
Motor thrust force:  0.2 × max_thrust     (pushing up)
Net force:           downward             (because 0.2 < 0.50 hover)
Altitude drop:       0.5 × net_accel × t² (significant in 0.4s)
```

In AirSim the hover thrust is lower so `thrust=0.2` was close enough to hover that the drop was minimal. In Gazebo it causes freefall.

### Position Controller Speed Difference

After the flip, the position controller must arrest the downward velocity and climb back. The default gains (`MPC_Z_VEL_P_ACC=4.0`) were designed for smooth position tracking, not aggressive post-flip recovery. In AirSim the drone barely dropped so slow gains were fine. In Gazebo the drone dropped significantly and slow gains couldn't recover fast enough.

### Summary of All Differences

| Parameter | AirSim Script | Gazebo Script | Reason |
|-----------|--------------|---------------|--------|
| Flip thrust | `0.2` | `0.55` | Must exceed `MPC_THR_HOVER=0.50` |
| Counter thrust | `0.8` | `0.9` | More force to arrest Gazebo's larger drop |
| Counter duration | `0.15s` | `0.20s` | Gazebo motor response slightly slower |
| Flip altitude | `15m` | `20m` | Extra 5m safety margin for larger drop |
| `MPC_Z_VEL_P_ACC` | `4.0` (default) | `8.0` | 2x faster altitude recovery |
| `MPC_Z_VEL_MAX_UP` | `3.0` (default) | `6.0` | Faster upward climb after flip |
| `MPC_ACC_UP_MAX` | default | `10.0` | Higher upward acceleration allowed |

---

## 3. Environment Setup

### Requirements

- WSL2 Ubuntu 20.04
- ROS Noetic (full desktop)
- MAVROS + MAVROS extras
- PX4-Autopilot v1.14.3
- Gazebo Classic 11 (installed by PX4 setup script)
- Python 3.8+

### Verify Gazebo is Installed

```bash
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x
```

### Build PX4 with Gazebo Support

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo_iris
```

This builds PX4 SITL and launches the Gazebo iris quadrotor model. The first build takes several minutes.

---

## 4. Startup Sequence

> **Key difference from AirSim:** No Windows needed. Everything runs in WSL2. No `PX4_SIM_HOST_ADDR` export required — Gazebo runs locally.

### Terminal 1: Start PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo_iris
```

Wait for Gazebo window to open and drone to appear, then wait for:

```
INFO  [commander] Ready for takeoff!
```

### Terminal 2: Set PX4 Parameters

In the PX4 console (`pxh>` prompt in Terminal 1):

```bash
# Standard flip parameters (same as AirSim)
param set COM_RC_IN_MODE 4
param set FD_FAIL_R 400
param set FD_FAIL_P 400
param set MC_ROLLRATE_MAX 1800
param set COM_OF_LOSS_T 5

# Gazebo-specific: faster vertical recovery
param set MPC_Z_VEL_P_ACC 8.0
param set MPC_Z_VEL_MAX_UP 6.0
param set MPC_ACC_UP_MAX 10.0

# Save all
param save
```

### Terminal 3: Start MAVROS

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch \
  fcu_url:="udp://:14550@127.0.0.1:14555" \
  tgt_system:=1 \
  tgt_component:=1
```

### Terminal 4: Run Flip Script

```bash
source /opt/ros/noetic/setup.bash
python3 drone_sitl_flip_gazebo.py
```

---

## 5. PX4 Parameters for Gazebo

### 5.1 Standard Flip Parameters (same as AirSim)

```bash
param set COM_RC_IN_MODE 4      # Disable RC requirement
param set FD_FAIL_R 400         # Disable roll failure detector
param set FD_FAIL_P 400         # Disable pitch failure detector
param set MC_ROLLRATE_MAX 1800  # Allow 30 rad/s flip roll rate
param set COM_OF_LOSS_T 5       # Offboard timeout 5 seconds
```

### 5.2 Gazebo-Specific Parameters (NEW — not needed for AirSim)

```bash
param set MPC_Z_VEL_P_ACC 8.0  # Vertical velocity P gain (default 4.0)
param set MPC_Z_VEL_MAX_UP 6.0 # Max upward velocity m/s (default 3.0)
param set MPC_ACC_UP_MAX 10.0  # Max upward acceleration (default 5.0)
param save
```

### 5.3 Why These Gazebo Parameters Are Needed

**`MPC_Z_VEL_P_ACC` — Vertical Velocity Proportional Gain:**

| Value | Behavior |
|-------|----------|
| `4.0` | Default — smooth position tracking, too slow for post-flip recovery |
| `6.0` | Moderate improvement |
| `8.0` | **Recommended for Gazebo flip** — 2x faster altitude recovery |
| `>10.0` | May cause oscillation |

**`MPC_Z_VEL_MAX_UP` — Maximum Upward Velocity:**

| Value | Behavior |
|-------|----------|
| `3.0` | Default — drone climbs slowly after flip |
| `6.0` | **Recommended** — allows 2x faster climb back to altitude |

**`MPC_ACC_UP_MAX` — Maximum Upward Acceleration:**

| Value | Behavior |
|-------|----------|
| `5.0` | Default — conservative |
| `10.0` | **Recommended** — allows aggressive thrust burst to recover altitude |

### 5.4 Key Parameter: MPC_THR_HOVER

This is the most important difference between AirSim and Gazebo:

```bash
param show MPC_THR_HOVER
# Gazebo iris: 0.5000 (50% thrust to hover)
# AirSim: effectively ~0.35 (lighter physics)
```

**This single value explains everything.** During the flip, thrust must stay at or above `MPC_THR_HOVER` to prevent freefall. That's why the Gazebo script uses `thrust=0.55` during the roll (vs `0.2` in AirSim).

**`MPC_THR_HOVER` values explained:**

| Value | Meaning |
|-------|---------|
| `< 0.3` | Very powerful motors — racing drone |
| `0.4–0.5` | Typical quadrotor |
| `0.5` | Gazebo iris model default |
| `> 0.6` | Heavy drone / underspecced motors |

---

## 6. Flip Script — Gazebo Tuned

Save as `drone_sitl_flip_gazebo.py`:

```python
#!/usr/bin/env python3
import rospy
import math
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

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

# Background publisher — NEVER stops (keeps PX4 in OFFBOARD mode)
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
            msg.type_mask     = 128  # IGNORE_ATTITUDE: use body rates + thrust
            msg.body_rate.x   = sp['roll_rate']
            msg.body_rate.y   = sp['pitch_rate']
            msg.body_rate.z   = sp['yaw_rate']
            msg.thrust        = sp['thrust']
            att_pub.publish(msg)
        r.sleep()

threading.Thread(target=bg_publish, daemon=True).start()

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
    """Sleep without blocking background publisher."""
    end = rospy.Time.now() + rospy.Duration(seconds)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        rospy.sleep(0.01)

def wait_until_stable(target_z=20.0, roll_threshold=5.0, timeout=30.0):
    """Wait until drone is level AND at target altitude before second flip."""
    rospy.loginfo(f"Waiting for stability (roll<{roll_threshold}deg z>{target_z-2:.0f}m)...")
    update_pos(0, 0, target_z)
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        roll_deg = math.degrees(get_roll())
        x, y, z  = get_pos()
        rospy.loginfo(f"  Stability: roll={roll_deg:.1f}deg  z={z:.2f}")
        if abs(roll_deg) < roll_threshold and z > target_z - 2.0:
            rospy.loginfo("Stable!")
            return True
        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("Timeout — proceeding.")
            return False
        rospy.sleep(0.3)

def flip(direction='right'):
    """
    Clover-style flip — GAZEBO TUNED version.

    Key differences from AirSim version:
      thrust during flip: 0.2  → 0.55  (must be > MPC_THR_HOVER=0.50)
      counter thrust:     0.8  → 0.9   (more aggressive altitude recovery)
      counter duration:   0.15 → 0.20  (Gazebo motor response slower)
      flip altitude:      15m  → 20m   (more safety margin)

    Why: Gazebo iris MPC_THR_HOVER=0.50. Commanding thrust=0.2 during flip
    means motors provide less than half the force needed to fight gravity.
    Result: drone freefalls during flip. Fix: thrust=0.55 > hover threshold.
    """
    sx, sy, sz = get_pos()
    rospy.loginfo(f"[FLIP {direction.upper()}] z={sz:.2f} roll={math.degrees(get_roll()):.1f}deg")

    # Phase 1: Thrust bump — same as AirSim
    # Gains altitude energy before flip
    rospy.loginfo("  Phase 1: Thrust bump (0.2s, thrust=1.0)...")
    update_rate(0, 0, 0, 1.0)
    sleep_publishing(0.2)

    # Phase 2: Roll rate
    # GAZEBO CHANGE: thrust 0.2 → 0.55
    # 0.55 > MPC_THR_HOVER(0.50) so drone does NOT freefall during rotation
    roll_rate = 30.0 if direction == 'right' else -30.0
    rospy.loginfo(f"  Phase 2: Rolling at {roll_rate} rad/s, thrust=0.55...")
    update_rate(roll_rate, 0, 0, 0.55)

    timeout = rospy.Time.now() + rospy.Duration(3.0)
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        roll = get_roll()
        rospy.loginfo(f"  roll={math.degrees(roll):.1f}deg")
        if abs(roll) > math.pi / 2:
            rospy.loginfo(f"  Flipped!")
            break
        rospy.sleep(0.01)

    # Phase 3: Counter-roll
    # GAZEBO CHANGE: thrust 0.8 → 0.9, duration 0.15 → 0.20
    counter_rate = -50.0 if direction == 'right' else 50.0
    rospy.loginfo(f"  Phase 3: Counter-roll ({counter_rate} rad/s, thrust=0.9, 0.20s)...")
    update_rate(counter_rate, 0, 0, 0.9)
    sleep_publishing(0.20)

    # Phase 4: Return to position
    # Works because MPC_Z_VEL_P_ACC=8.0 and MPC_Z_VEL_MAX_UP=6.0
    # give faster vertical recovery
    rospy.loginfo(f"  Phase 4: Recovering to z={sz:.2f}...")
    update_pos(sx, sy, sz)
    sleep_publishing(3.0)

    rospy.loginfo(f"[FLIP {direction.upper()}] Done! z={get_pos()[2]:.2f}")

# Main
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not current_state.connected:
    rospy.sleep(0.1)
rospy.loginfo("Connected!")

rospy.loginfo("Pre-streaming setpoints for 5 seconds...")
update_pos(0, 0, 3)
sleep_publishing(5.0)

offb = SetModeRequest()
offb.custom_mode = 'OFFBOARD'
arm = CommandBoolRequest()
arm.value = True
last_req    = rospy.Time.now()
rate        = rospy.Rate(20)
phase       = "TAKEOFF"
phase_start = rospy.Time.now()

rospy.loginfo("Starting flight (Gazebo-tuned)...")

while not rospy.is_shutdown():

    if current_state.mode != "OFFBOARD" and \
       (rospy.Time.now() - last_req) > rospy.Duration(5.0):
        if set_mode_client.call(offb).mode_sent:
            rospy.loginfo("OFFBOARD enabled!")
        last_req = rospy.Time.now()
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
        update_pos(0, 0, 20)      # 20m vs 15m in AirSim
        if z > 18.0:
            rospy.loginfo(f"Flip altitude reached. z={z:.2f}")
            phase = "STABILIZE_1"
            phase_start = rospy.Time.now()

    elif phase == "STABILIZE_1":
        update_pos(0, 0, 20)
        if (rospy.Time.now() - phase_start).to_sec() > 3.0:
            rospy.loginfo("Ready for first flip!")
            phase = "FLIP_RIGHT"

    elif phase == "FLIP_RIGHT":
        flip('right')
        phase = "STABILIZE_2"

    elif phase == "STABILIZE_2":
        wait_until_stable(target_z=20.0, roll_threshold=5.0, timeout=30.0)
        rospy.loginfo("Ready for second flip!")
        phase = "FLIP_LEFT"

    elif phase == "FLIP_LEFT":
        flip('left')
        phase = "DONE"

    elif phase == "DONE":
        update_pos(0, 0, 3)
        rospy.loginfo(f"All flips complete! z={z:.2f} roll={roll_deg:.1f}deg")
        sleep_publishing(5.0)
        break

    rate.sleep()

rospy.loginfo("Script complete.")
```

---

## 7. Tuning Reference

### 7.1 Complete Parameter Diff: AirSim vs Gazebo

| Parameter | AirSim | Gazebo | Set Where |
|-----------|--------|--------|-----------|
| `COM_RC_IN_MODE` | `4` | `4` | PX4 console |
| `FD_FAIL_R` | `400` | `400` | PX4 console |
| `FD_FAIL_P` | `400` | `400` | PX4 console |
| `MC_ROLLRATE_MAX` | `1800` | `1800` | PX4 console |
| `COM_OF_LOSS_T` | `5` | `5` | PX4 console |
| `MPC_Z_VEL_P_ACC` | `4.0` (default) | **`8.0`** | PX4 console |
| `MPC_Z_VEL_MAX_UP` | `3.0` (default) | **`6.0`** | PX4 console |
| `MPC_ACC_UP_MAX` | `5.0` (default) | **`10.0`** | PX4 console |

### 7.2 Script Value Diff: AirSim vs Gazebo

| Script Value | AirSim | Gazebo | Why |
|-------------|--------|--------|-----|
| Thrust bump duration | `0.2s` | `0.2s` | Same |
| **Flip thrust** | **`0.2`** | **`0.55`** | Must exceed `MPC_THR_HOVER=0.50` |
| **Counter thrust** | **`0.8`** | **`0.9`** | More force to arrest larger drop |
| **Counter duration** | **`0.15s`** | **`0.20s`** | Gazebo motor response slower |
| **Flip altitude** | **`15m`** | **`20m`** | Extra safety margin |

### 7.3 If Recovery Is Still Too Slow

Try increasing in this order:

```bash
# Option 1: More aggressive vertical gains
param set MPC_Z_VEL_P_ACC 10.0   # increase from 8.0
param set MPC_Z_VEL_MAX_UP 8.0   # increase from 6.0
param save

# Option 2: Higher flip altitude
# In script: change 20m to 25m

# Option 3: Increase flip thrust slightly
# In script: change 0.55 to 0.60
```

### 7.4 If Second Flip Spins Multiple Times

```bash
# Increase stability check roll threshold
# wait_until_stable(roll_threshold=3.0)   # stricter: was 5.0

# Or increase stabilize time
# wait_until_stable(timeout=45.0)         # more time to stabilize
```

---

## 8. Troubleshooting

| Error | Fix |
|-------|-----|
| `Drone crashes during flip` | `MPC_Z_VEL_P_ACC` not set. Run `param set MPC_Z_VEL_P_ACC 8.0` and `param save` |
| `Flip thrust too low` | Check `param show MPC_THR_HOVER` — flip thrust must be `> MPC_THR_HOVER` |
| `Gazebo not starting` | Check `gazebo --version`. If missing: `sudo apt install gazebo11 libgazebo11-dev` |
| `Drone oscillates after flip` | `MPC_Z_VEL_P_ACC` too high. Reduce from 8.0 to 6.0 |
| `Second flip crashes` | `wait_until_stable` timeout too short. Increase to 45s |
| `param save not persisting` | Run `make clean` then rebuild. SITL resets params on clean build |
| `Gazebo window freezes` | Reduce simulation speed: `export PX4_SIM_SPEED_FACTOR=0.5` before make |
| `OFFBOARD rejected` | Not pre-streaming. `sleep_publishing(5.0)` must complete before OFFBOARD |

### Key Diagnostic Commands

```bash
# Check hover thrust (flip thrust must exceed this)
param show MPC_THR_HOVER

# Check vertical controller gains
param show MPC_Z_VEL_P_ACC
param show MPC_Z_VEL_MAX_UP
param show MPC_ACC_UP_MAX

# Check EKF2 status
ekf2 status

# Reset SITL params to default (WARNING: clears all your settings)
make clean
```

---

## Appendix: Quick Reference

### Terminal Commands

```bash
# Terminal 1: PX4 SITL + Gazebo (no Windows, no export needed)
cd ~/PX4-Autopilot
make px4_sitl gazebo_iris

# PX4 console — standard flip params
param set COM_RC_IN_MODE 4
param set FD_FAIL_R 400
param set FD_FAIL_P 400
param set MC_ROLLRATE_MAX 1800
param set COM_OF_LOSS_T 5

# PX4 console — Gazebo-specific params
param set MPC_Z_VEL_P_ACC 8.0
param set MPC_Z_VEL_MAX_UP 6.0
param set MPC_ACC_UP_MAX 10.0
param save

# Terminal 2: MAVROS (same as AirSim)
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14555" tgt_system:=1 tgt_component:=1

# Terminal 3: Flip script
source /opt/ros/noetic/setup.bash
python3 drone_sitl_flip_gazebo.py
```

### The Single Most Important Insight

```
MPC_THR_HOVER = 0.50  (Gazebo iris)

During flip, thrust must be > 0.50 or drone freefalls.
Script uses thrust = 0.55 — just above hover threshold.

This is the ONLY reason the AirSim script fails in Gazebo.
```

### What to Expect on Real Drone

Your real drone will behave more like Gazebo than AirSim:
- `MPC_THR_HOVER` on real hardware is typically `0.40–0.60`
- Check with `param show MPC_THR_HOVER` after arming
- Set flip thrust to `MPC_THR_HOVER + 0.05` as starting point
- Increase `MPC_Z_VEL_P_ACC` to `6.0–8.0` for fast recovery

---

*Generated: May 2026 | PX4 v1.14.3 | ROS Noetic | Ubuntu 20.04 WSL2 | Gazebo Classic 11*

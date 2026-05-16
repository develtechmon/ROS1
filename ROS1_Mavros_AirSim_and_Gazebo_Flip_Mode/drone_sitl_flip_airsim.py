#!/usr/bin/env python3
import rospy
import math
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# ── State ─────────────────────────────────────────────────────────────────────
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

# ── Shared setpoint ───────────────────────────────────────────────────────────
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

# ── Background publisher — NEVER STOPS ───────────────────────────────────────
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
            msg.type_mask     = 128  # IGNORE_ATTITUDE — use body rates + thrust
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
    Prevents second flip from starting with residual angular velocity.
    """
    rospy.loginfo(f"Waiting for stability (roll<{roll_threshold}° z>{target_z-2:.0f}m)...")
    update_pos(0, 0, target_z)
    start = rospy.Time.now()

    while not rospy.is_shutdown():
        roll_deg = math.degrees(get_roll())
        x, y, z  = get_pos()
        rospy.loginfo(f"  Stability check: roll={roll_deg:.1f}° z={z:.2f}")

        if abs(roll_deg) < roll_threshold and z > target_z - 2.0:
            rospy.loginfo("Stable! Ready for next flip.")
            return True

        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("Stability timeout — proceeding anyway.")
            return False

        rospy.sleep(0.3)

# ── Flip ──────────────────────────────────────────────────────────────────────
def flip(direction='right'):
    """
    Clover-style flip using MAVROS AttitudeTarget body rates.
    type_mask=128 (IGNORE_ATTITUDE) tells PX4 to use body rates, not quaternion.
    """
    sx, sy, sz = get_pos()
    rospy.loginfo(f"[FLIP {direction.upper()}] Starting from z={sz:.2f} roll={math.degrees(get_roll()):.1f}°")

    # Phase 1: Thrust bump — gain energy before flip
    # Clover: set_rates(thrust=1), sleep(0.2)
    rospy.loginfo("  Phase 1: Thrust bump...")
    update_rate(0, 0, 0, 1.0)
    sleep_publishing(0.2)

    # Phase 2: Full roll rate
    # Clover: set_rates(roll_rate=30, thrust=0.2)
    roll_rate = 30.0 if direction == 'right' else -30.0
    rospy.loginfo(f"  Phase 2: Rolling at {roll_rate} rad/s...")
    update_rate(roll_rate, 0, 0, 0.2)

    # Wait until flipped past 90 degrees
    # Clover simplified: flipped = abs(roll) > PI/2
    timeout = rospy.Time.now() + rospy.Duration(3.0)
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        roll = get_roll()
        rospy.loginfo(f"  roll={math.degrees(roll):.1f}°")
        if abs(roll) > math.pi / 2:
            rospy.loginfo(f"  Flipped! roll={math.degrees(roll):.1f}°")
            break
        rospy.sleep(0.01)

    # Phase 3: Counter-roll to stop rotation
    # Clover: set_rates(roll_rate=-50, thrust=0.8), sleep(0.15)
    counter_rate = -50.0 if direction == 'right' else 50.0
    rospy.loginfo(f"  Phase 3: Counter-roll at {counter_rate} rad/s...")
    update_rate(counter_rate, 0, 0, 0.8)
    sleep_publishing(0.15)

    # Phase 4: Return to start position
    # Clover: set_position(x=start.x, y=start.y, z=start.z)
    rospy.loginfo(f"  Phase 4: Recovering to z={sz:.2f}...")
    update_pos(sx, sy, sz)
    sleep_publishing(3.0)

    rospy.loginfo(f"[FLIP {direction.upper()}] Done! roll={math.degrees(get_roll()):.1f}°")

# ── Main ──────────────────────────────────────────────────────────────────────
rospy.loginfo("Waiting for FCU connection...")
while not rospy.is_shutdown() and not current_state.connected:
    rospy.sleep(0.1)
rospy.loginfo("Connected!")

# Stream setpoints before requesting OFFBOARD (mandatory)
rospy.loginfo("Pre-streaming setpoints for 5 seconds...")
update_pos(0, 0, 3)
sleep_publishing(5.0)

# OFFBOARD + ARM
offb = SetModeRequest()
offb.custom_mode = 'OFFBOARD'
arm = CommandBoolRequest()
arm.value = True
last_req = rospy.Time.now()

rate  = rospy.Rate(20)
phase = "TAKEOFF"
phase_start = rospy.Time.now()

rospy.loginfo("Starting flight...")

while not rospy.is_shutdown():

    # Keep OFFBOARD mode and armed
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

    # ── TAKEOFF ───────────────────────────────────────────────────────────────
    if phase == "TAKEOFF":
        update_pos(0, 0, 3)
        if current_state.armed and z > 2.5:
            rospy.loginfo(f"Takeoff complete. z={z:.2f}")
            phase = "CLIMB"

    # ── CLIMB TO FLIP ALTITUDE ────────────────────────────────────────────────
    elif phase == "CLIMB":
        update_pos(0, 0, 15)
        if z > 13.0:
            rospy.loginfo(f"Flip altitude reached. z={z:.2f}")
            phase = "STABILIZE_1"
            phase_start = rospy.Time.now()

    # ── STABILIZE BEFORE FIRST FLIP ───────────────────────────────────────────
    elif phase == "STABILIZE_1":
        update_pos(0, 0, 15)
        if (rospy.Time.now() - phase_start).to_sec() > 3.0:
            rospy.loginfo("Ready for first flip!")
            phase = "FLIP_RIGHT"

    # ── FIRST FLIP (RIGHT) ────────────────────────────────────────────────────
    elif phase == "FLIP_RIGHT":
        flip('right')
        phase = "STABILIZE_2"

    # ── STABILIZE BETWEEN FLIPS ───────────────────────────────────────────────
    elif phase == "STABILIZE_2":
        # Actively wait until drone is confirmed stable
        # Prevents residual angular velocity from ruining second flip
        wait_until_stable(target_z=15.0, roll_threshold=5.0, timeout=30.0)
        rospy.loginfo("Ready for second flip!")
        phase = "FLIP_LEFT"

    # ── SECOND FLIP (LEFT) ────────────────────────────────────────────────────
    elif phase == "FLIP_LEFT":
        flip('left')
        phase = "DONE"

    # ── DONE ──────────────────────────────────────────────────────────────────
    elif phase == "DONE":
        update_pos(0, 0, 3)
        rospy.loginfo(f"All flips complete! z={z:.2f} roll={roll_deg:.1f}°")
        sleep_publishing(5.0)
        break

    rate.sleep()

rospy.loginfo("Script complete.")

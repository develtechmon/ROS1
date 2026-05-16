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
            msg.type_mask     = 128
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
    end = rospy.Time.now() + rospy.Duration(seconds)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        rospy.sleep(0.01)

def wait_until_stable(target_z=15.0, roll_threshold=5.0, timeout=30.0):
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
    sx, sy, sz = get_pos()
    rospy.loginfo(f"[FLIP {direction.upper()}] z={sz:.2f} roll={math.degrees(get_roll()):.1f}deg")

    # Phase 1: Thrust bump — same as AirSim
    rospy.loginfo("  Phase 1: Thrust bump...")
    update_rate(0, 0, 0, 1.0)
    sleep_publishing(0.2)                         # same as AirSim

    # Phase 2: Roll rate
    # CHANGE 1: thrust 0.2 → 0.55
    # AirSim hover thrust ~0.35 so 0.2 was fine (below hover = minor drop)
    # Gazebo hover thrust = 0.50 so 0.2 = freefall
    # 0.55 = just above hover = drone maintains altitude during flip
    roll_rate = 30.0 if direction == 'right' else -30.0
    rospy.loginfo(f"  Phase 2: Rolling at {roll_rate} rad/s, thrust=0.55...")
    update_rate(roll_rate, 0, 0, 0.55)            # was 0.2 in AirSim

    timeout = rospy.Time.now() + rospy.Duration(3.0)
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        roll = get_roll()
        rospy.loginfo(f"  roll={math.degrees(roll):.1f}deg")
        if abs(roll) > math.pi / 2:
            rospy.loginfo(f"  Flipped!")
            break
        rospy.sleep(0.01)

    # Phase 3: Counter-roll
    # CHANGE 2: thrust 0.8 → 0.9 (more aggressive recovery)
    # CHANGE 3: duration 0.15 → 0.20 (Gazebo motor response slightly slower)
    counter_rate = -50.0 if direction == 'right' else 50.0
    rospy.loginfo(f"  Phase 3: Counter-roll...")
    update_rate(counter_rate, 0, 0, 0.9)          # was 0.8 in AirSim
    sleep_publishing(0.20)                         # was 0.15 in AirSim

    # Phase 4: Return to position — same as AirSim
    # Now works properly because:
    # - MPC_Z_VEL_P_ACC=8.0 makes position controller 2x faster
    # - MPC_Z_VEL_MAX_UP=6.0 allows faster upward climb
    # - Drone didn't freefall during flip (thrust=0.55 > hover=0.50)
    rospy.loginfo(f"  Phase 4: Recovering to z={sz:.2f}...")
    update_pos(sx, sy, sz)
    sleep_publishing(3.0)                          # same as AirSim

    rospy.loginfo(f"[FLIP {direction.upper()}] Done! z={get_pos()[2]:.2f}")

# ── Main ──────────────────────────────────────────────────────────────────────
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
        # CHANGE 4: climb to 20m (was 15m in AirSim)
        # Extra 5m safety margin for altitude drop during flip
        update_pos(0, 0, 20)
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

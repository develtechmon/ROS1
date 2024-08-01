from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink.dialects.v10 import ardupilotmega
from pymavlink import mavutil
import dronekit
import time
import math

# Set up option parsing to get connection string
import argparse

connection_string = '/dev/ttyAMA0,921600'

'''
Class name: Custom_DroneKit_Vehicle
Description: Add some custom features to original DroneKit Vehicle class
'''
class Custom_DroneKit_Vehicle(dronekit.Vehicle):
	def __init__(self, *args):
		super(Custom_DroneKit_Vehicle, self).__init__(*args)
		
		self._ekf_predposhorizrel = False
		@self.on_message('EKF_STATUS_REPORT')
		def listener(self, name, m):
			# boolean: EKF's predicted horizontal position (relative) estimate is good
			self._ekf_predposhorizrel = (m.flags & ardupilotmega.EKF_PRED_POS_HORIZ_REL) > 0
			self.notify_attribute_listeners('is_armable', self.is_armable, cache=True)
	
	'''
	Property name: is_armable
	Description: override the is_armable property in dronekit.Vehicle, intended for
	             indoor OF + RF autonomous missions. 
	'''
	@property
	def is_armable(self):
		return self.mode != 'INITIALISING' and self.rangefinder.distance and self._ekf_predposhorizrel

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True,vehicle_class=Custom_DroneKit_Vehicle)
print("Virtual Copter is Ready")

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def hover():
    print("Set the channel overrides to Loiter")
    vehicle.channels.overrides['3'] = 1500
    msg = vehicle.message_factory.command_long_encode(
            0,0,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,
            0,0,0,0,
            0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(vehicle.mode)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)
    
def arm_and_takeoff_nogps(aTargetAltitude):
    vstate = vehicle.system_status.state
    if vstate != "STANDBY":
        print("Vehicle is not ready for arming !")
        return False
    
    # Change Flight Mode to GUIDE_NO_GPS
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    # Takeoff up to 0.5m and hover
    while True:
        current_altitude = vehicle.rangefinder.distance
        print("Hovering at %f meters.\n" % current_altitude)
        if current_altitude >= aTargetAltitude:
            break
        else:
            thrust = 0.55
        time.sleep(0.2)
        set_attitude(thrust=thrust)

# Take off 2.5m in GUIDED_NOGPS mode.
arm_and_takeoff_nogps(0.3)

print("Hold position for 5 seconds")
#set_attitude(duration = 5)
hover()

#print("Setting LAND mode...")
#vehicle.mode = VehicleMode("LAND")
#time.sleep(1)

# Close vehicle object before exiting script
#print("Close vehicle object")
#vehicle.close()

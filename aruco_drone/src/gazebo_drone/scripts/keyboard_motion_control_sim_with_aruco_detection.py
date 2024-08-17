#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import ros_numpy as rnp

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink.dialects.v10 import ardupilotmega
from pymavlink import mavutil
import dronekit
import time
import math
import keyboard
import threading
import state
import cv2
import numpy as np

# Set up option parsing to get connection string
import argparse

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

connection_string = 'tcp:127.0.0.1:5763'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, vehicle_class=Custom_DroneKit_Vehicle)
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
vehicle.parameters['LAND_SPEED'] = 20 ##Descent speed of 30cm/s
print("Virtual Copter is Ready")

velocity = 0.5 # m/s
takeoff_height = 5 # m

## Variables ##
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72 ## arucoID default is 72
marker_size = 20  ## CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)

found_count = 0
not_found_count = 0

## Camera Intrinsic ## 
dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

## Functions ##
time_last = 0
time_to_wait = .1 ## 100ms

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0
    )
    for _ in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
        
def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
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
    print("Current Mode is : " + state.get_system_state())
    print("Set the channel overrides to Loiter")
    
    while True:
        vehicle.channels.overrides['3'] = 1500
        msg = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(msg)
        vehicle.flush()
        
        time.sleep(0.1)
        

        if (state.get_system_state() == "land"):
            land()
            break
        
def msg_receiver(message):
    global not_found_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
        
        print("ID: =" + str(ids))

        try:
            if ids is not None:
                if ids[0] == id_to_find: 
                        #corners_single=[corners[ids_array_index]]
                        #corners_single_np = np.asarray(corners_single)
                        
                        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                        (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                        x = '{:.2f}'.format(tvec[0])
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])
                        
                        x_sum = 0
                        y_sum = 0
                    
                        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
                        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
                        
                        # This will give us centre point of the aruco marker in the frame itself
                        #cx = x_sum*.25
                        #cy = y_sum*.25
                        cx = x_sum / 4
                        cy = y_sum / 4
                    
                        # X-angle and Y-angle which used by mavlink message
                        x_ang = (cx - horizontal_res*0.5)*(horizontal_fov/horizontal_res)
                        y_ang = (cy - vertical_res*0.5)*(vertical_fov/vertical_res)
                        
                        # if vehicle.mode != 'LAND':
                        #     vehicle.mode = VehicleMode('LAND')
                        #     while vehicle.mode != 'LAND':
                        #         time.sleep(1)
                        #     print("Vehicle in LAND mode")
                        #     send_land_message(x_ang,y_ang)
                        # else:
                        #     send_land_message(x_ang,y_ang)
                        #     pass

                        marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z

                        aruco.drawDetectedMarkers(np_data, corners)
                        aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec,10)

                        cv2.putText(np_data, marker_position, (10,50),0,.5,(255,0,0), thickness=2)
                        #print(marker_position)
                        
                        # We print the information of how far away is the aruco marker from the centre
                        print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
                        
                        # How many aruco has been found
                        print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(not_found_count))
                        
                        # Marker position in cm
                        print("MARKER POSITION: x=" +x+" y= "+y+" z="+z)
                        
                        print(" ")
                    
                        found_count = found_count + 1                
                
                else:
                    not_found_count=not_found_count+1
            else:
                not_found_count=not_found_count+1

        except Exception as e:
            print("Target not found")
            print(e)
            not_found_count = not_found_count + 1
        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    
    else:
        return None

            
def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    if yaw_angle is None:
        yaw_angle = vehicle.attitude.yaw
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                 yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                 thrust=0.5, duration=0):
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def arm_and_takeoff_nogps(aTargetAltitude):
    vstate = vehicle.system_status.state
    if vstate != "STANDBY":
        print("Vehicle is not ready for arming !")
        return False
    
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while True:
        current_altitude = vehicle.rangefinder.distance
        print("Hovering at %f meters.\n" % current_altitude)
        if current_altitude >= aTargetAltitude - 0.1:
            print("Reached altitude !")
            break
        else:
            thrust = 0.6
        time.sleep(0.2)
        set_attitude(thrust=thrust)
    state.set_system_state("loiter")

def land():
    vehicle.mode = VehicleMode("LAND")
    
def keyboard_control():
    while True:
        try:
            if keyboard.is_pressed('w'):
                print("Forward")
                vehicle.channels.overrides['2'] = 1400  # Pitch backward

            elif keyboard.is_pressed('s'):
                print("Backward")
                vehicle.channels.overrides['2'] = 1600  # Pitch forward

            elif keyboard.is_pressed('a'):
                print("Left")
                vehicle.channels.overrides['1'] = 1400  # Roll left

            elif keyboard.is_pressed('d'):
                print("Right")
                vehicle.channels.overrides['1'] = 1600  # Roll right
                
            elif keyboard.is_pressed('q'):
                print("Yaw Left")
                vehicle.channels.overrides['4'] = 1400  # Yaw left
            
            elif keyboard.is_pressed('e'):
                print("Yaw Right")
                vehicle.channels.overrides['4'] = 1600  # Yaw right
                
            elif keyboard.is_pressed('r'): # Land 
                print("Land")
                vehicle.channels.overrides['3'] = 1000
                state.set_system_state("land")
                
            elif keyboard.is_pressed('t'): # Takeoff 
                print("Takeoff")
                state.set_system_state("takeoff")
                control()  
            
            elif keyboard.is_pressed('space'): # Brake
                print("Brake")
                vehicle.channels.overrides['1'] = 1500  # Center roll
                vehicle.channels.overrides['2'] = 1500  # Center pitch
                vehicle.channels.overrides['4'] = 1500  # Center yaw
                    
            else:
                vehicle.channels.overrides['1'] = 1500  # Center roll
                vehicle.channels.overrides['2'] = 1500  # Center pitch
                vehicle.channels.overrides['4'] = 1500  # Center yaw
                
            time.sleep(0.05)
            
        except:
            pass

def subscriber():
    # rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()
    
def control():
    while True:
        print("Current Mode is : " + state.get_system_state())
        if (state.get_system_state() == "takeoff"):
            arm_and_takeoff_nogps(0.5)
            time.sleep(1)
        
        elif (state.get_system_state() == "loiter"):
            hover_thread = threading.Thread(target=hover)
            control_thread = threading.Thread(target=keyboard_control)

            control_thread.start()
            hover_thread.start()
            
            hover_thread.join()
            control_thread.join()
            

if __name__ == "__main__":
    
    state.set_system_state("takeoff")    
    rospy.init_node('drone_node', anonymous=False)
    
    # Start subscriber in the main thread
    subscriber_thread = threading.Thread(target=subscriber)
    subscriber_thread.start()

    # Start control logic in a separate thread
    control_thread = threading.Thread(target=control)
    control_thread.start()
    
    # Keep the main thread alive
    try:
        subscriber_thread.join()
        control_thread.join()
    except rospy.ROSInterruptException:
        pass

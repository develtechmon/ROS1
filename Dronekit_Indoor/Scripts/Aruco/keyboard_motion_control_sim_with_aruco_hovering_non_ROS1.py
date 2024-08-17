#!/usr/bin/env python

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

import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

import os
import platform
import sys

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

# For Simulation 
#connection_string = 'tcp:127.0.0.1:5763'

# For RPI IP Address
#connection_string = '192.168.8:5763'
connection_string = '192.168.8:14553'

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
# dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
# camera_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]
# np_camera_matrix = np.array(camera_matrix)
# np_dist_coeff = np.array(dist_coeff)

calib_path=r"/home/jlukas/Desktop/My_Project/ROS1/Dronekit_Indoor/Scripts/Aruco/calibration_files/"
np_camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
np_dist_coeff   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

## Functions ##
time_last = 0
time_to_wait = .1 ## 100ms

## PID
P = 0.5
I = 0.2
D = 0.00
Roll_PError = 0
Pitch_PError = 0

## Camera Resolution
width=640
height=480

cap = WebcamVideoStream(src=0).start()

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
        
def adjust_position_my_pid(cx, cy,z=0):
    global Roll_PError, Pitch_PError  # Declare the variables as global

    # Calculate errors based on the difference from the center of the frame
    roll_error = cx 
    pitch_error = cy
    
    speed_roll = P * roll_error + I * (roll_error - Roll_PError)
    speed_pitch = P * pitch_error + I * (pitch_error - Pitch_PError)
    
    # Adjust pitch (channel 2) based on y_ang
    pitch_pwm = 1500 + int(speed_pitch * 500)  # 500 is a scaling factor to convert angle to PWM
    pitch_pwm = max(1400, min(1600, pitch_pwm))  # Ensure PWM is within valid range

    # Adjust roll (channel 1) based on x_ang
    roll_pwm = 1500 + int(speed_roll * 500)  # 500 is a scaling factor to convert angle to PWM
    roll_pwm = max(1400, min(1600, roll_pwm))  # Ensure PWM is within valid range
    
    #throttle_pwm = 1500 + int(throttle_output * 500)
    #throttle_pwm = max(1400, min(1600, throttle_pwm))

    Roll_PError = roll_error
    Pitch_PError = pitch_error

    # Set the channel overrides
    vehicle.channels.overrides['1'] = roll_pwm
    vehicle.channels.overrides['2'] = pitch_pwm

    print(f"Adjusted RC Overrides -> Roll: {roll_pwm}, Pitch: {pitch_pwm}")
    print(f"Centre-> Cx: {cx}, Cy: {cy}")
    print(f"PID-> speed_roll: {speed_roll}, speed_pitch: {speed_pitch}")

def adjust_position_simple_pid(x_ang, y_ang, z_dist): # No Working at the moment
    # Ensure the inputs are converted to float
    x_ang = float(x_ang)
    y_ang = float(y_ang)
    z_dist = float(z_dist)

    # Calculate the error from the center
    roll_error = x_ang
    pitch_error = y_ang
    altitude_error = z_dist - 1.0  # Assuming we want to maintain 1 meter altitude

    # Get PID outputs
    roll_output = pid_roll(roll_error)
    pitch_output = pid_pitch(pitch_error)
    throttle_output = pid_throttle(altitude_error)

    # Convert PID output to RC PWM values
    roll_pwm = 1500 + int(roll_output * 500)  # Scale factor as needed
    pitch_pwm = 1500 + int(pitch_output * 500)
    throttle_pwm = 1500 + int(throttle_output * 500)

    # Clamp PWM values to valid range
    roll_pwm = max(1400, min(1600, roll_pwm))
    pitch_pwm = max(1400, min(1600, pitch_pwm))
    throttle_pwm = max(1400, min(1600, throttle_pwm))
   
    # Set the channel overrides
    vehicle.channels.overrides['1'] = roll_pwm
    vehicle.channels.overrides['2'] = pitch_pwm
    #vehicle.channels.overrides['3'] = throttle_pwm

    print(f"Adjusted RC Overrides -> Roll: {roll_pwm}, Pitch: {pitch_pwm}, Throttle: {throttle_pwm}")

def adjust_position_no_pid(x_ang, y_ang, z_dist):
    # Ensure the inputs are converted to float
    x_ang = float(x_ang)
    y_ang = float(y_ang)
    z_dist = float(z_dist)

    # Map x_ang and y_ang to RC channel values.
    # Assuming that 1500 is the neutral position, and 1000/2000 are the extremes.

    # Adjust pitch (channel 2) based on y_ang
    pitch_pwm = 1500 + int(y_ang * 500)  # 500 is a scaling factor to convert angle to PWM
    pitch_pwm = max(1400, min(1600, pitch_pwm))  # Ensure PWM is within valid range

    # Adjust roll (channel 1) based on x_ang
    roll_pwm = 1500 + int(x_ang * 500)  # 500 is a scaling factor to convert angle to PWM
    roll_pwm = max(1400, min(1600, roll_pwm))  # Ensure PWM is within valid range

    # Optional: Adjust throttle (channel 3) based on z_dist, if needed
    # For example, maintain a certain distance from the marker by adjusting the throttle
    # Here, assume you want to maintain a specific altitude (e.g., 1 meter)
    desired_altitude = 1.0  # meters
    altitude_error = z_dist - desired_altitude
    throttle_pwm = 1500 + int(altitude_error * 500)  # Adjust as necessary
    throttle_pwm = max(1000, min(2000, throttle_pwm))

    # Set the channel overrides
    vehicle.channels.overrides['1'] = roll_pwm
    vehicle.channels.overrides['2'] = pitch_pwm
    #vehicle.channels.overrides['3'] = throttle_pwm

    print(f"Adjusted RC Overrides -> Roll: {roll_pwm}, Pitch: {pitch_pwm}, Throttle: {throttle_pwm}")
    print(f"x_ang -> x_ang: {x_ang}, y_ang: {y_ang}")
    print(f"PWM -> pitch_pwm: {pitch_pwm}, roll_pwm: {roll_pwm}")

def msg_receiver():
    global not_found_count, found_count, time_last, time_to_wait, id_to_find
    
    while True:
        message = cap.read()
        if time.time() - time_last > time_to_wait:
            np_data = np.array(message)
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

            ids = ''
            (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
            
            cv2.circle(np_data, (width//2, height//2), 8 , (0,0,255), cv2.FILLED)

            print("Detected ID: =" + str(ids))

            try:
                if ids is not None:
                    if ids[0][0] == id_to_find: # To extract bracket of [id] and left only the integer id for correct detection
                        print("Operated ID: " + str(ids[0][0])) 
                        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                        (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                        x = '{:.2f}'.format(tvec[0])
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])

                        # Calculate the center of the Aruco marker
                        x_sum = sum([corner[0] for corner in corners[0][0]])
                        y_sum = sum([corner[1] for corner in corners[0][0]])
                        
                        cx = x_sum / 4
                        cy = y_sum / 4

                        # Calculate the error between the marker's center and the image center
                        x_error = cx - (horizontal_res / 2)
                        y_error = cy - (vertical_res / 2)

                        # Convert these errors into angles for drone adjustment
                        x_ang = x_error * (horizontal_fov / horizontal_res)
                        y_ang = y_error * (vertical_fov / vertical_res)

                        marker_position = f'MARKER POSITION: x={x} y={y} z={z}'

                        aruco.drawDetectedMarkers(np_data, corners)
                        aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)

                        cv2.putText(np_data, marker_position, (10, 50), 0, .5, (255, 0, 0), thickness=2)
                        
                        print(f"X CENTER PIXEL: {cx} Y CENTER PIXEL: {cy}")
                        print(f"FOUND COUNT: {found_count} NOTFOUND COUNT: {not_found_count}")
                        print(f"MARKER POSITION: x={x} y={y} z={z}\n")
                        print(f"ERROR POSITION: Error X={x_error} Error Y={y_error}\n")

                        # Adjust the drone's position using the calculated angles
                        
                        # Working 
                        # To adjust the drone position hovering without PID controller but still work very well
                        adjust_position_no_pid(x_ang, y_ang, z)
                        
                        # To adjust the drone position hovering with PID controller, work but might need to tune the PID
                        #adjust_position_my_pid(x_error,y_error,z)
                        
                        # Work in progress
                        # To adjust the drone position hovering with PID controller using simple pid library, can't get fix position. 
                        #adjust_position_simple_pid(x_ang, y_ang,z)
                        
                        found_count += 1                
                    else:
                        not_found_count += 1
                else:
                    not_found_count += 1

            except Exception as e:
                print("Target not found")
                print(e)
                not_found_count += 1
                
              # Show the frame
            cv2.imshow("Frame", np_data)

            # Press 'q' to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
            # newimg_pub.publish(new_msg)
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
    #sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    #rospy.spin()
    msg_receiver()        
    
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
    except:
        pass

#!/usr/bin/env python

from dronekit import connect, VehicleMode
from pymavlink.dialects.v10 import ardupilotmega
from pymavlink import mavutil
import dronekit
import time
import math
import threading
import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import state

# Class to handle custom vehicle functionalities
class Custom_DroneKit_Vehicle(dronekit.Vehicle):
    def __init__(self, *args):
        super(Custom_DroneKit_Vehicle, self).__init__(*args)
        self._ekf_predposhorizrel = False
        
        @self.on_message('EKF_STATUS_REPORT')
        def listener(self, name, m):
            self._ekf_predposhorizrel = (m.flags & ardupilotmega.EKF_PRED_POS_HORIZ_REL) > 0
            self.notify_attribute_listeners('is_armable', self.is_armable, cache=True)
    
    @property
    def is_armable(self):
        return self.mode != 'INITIALISING' and self.rangefinder.distance and self._ekf_predposhorizrel

# Connection settings
connection_string = '192.168.8.146:14553'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, vehicle_class=Custom_DroneKit_Vehicle)
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 20
print("Vehicle is Ready")

# Aruco marker setup
id_to_find = 72
marker_size = 20  # cm
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Camera setup
horizontal_res = 640
vertical_res = 480
horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)

# Load camera calibration
calib_path = r"/home/jlukas/Desktop/My_Project/ROS1/Dronekit_Indoor/Scripts/Aruco/calibration_files/"
np_camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
np_dist_coeff = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

# Webcam setup
cap = WebcamVideoStream(src=0).start()

# PID parameters
P = 0.5
I = 0.2
D = 0.00
Roll_PError = 0
Pitch_PError = 0

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """Convert degrees to quaternions"""
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
    while state.get_system_state() == "loiter":
        vehicle.channels.overrides['3'] = 1500
        msg = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.1)

def adjust_position_no_pid(x_ang, y_ang, z_dist):
    roll_pwm = 1500 + int(x_ang * 500)
    pitch_pwm = 1500 + int(y_ang * 500)
    roll_pwm = max(1400, min(1600, roll_pwm))
    pitch_pwm = max(1400, min(1600, pitch_pwm))
    
    vehicle.channels.overrides['1'] = roll_pwm
    vehicle.channels.overrides['2'] = pitch_pwm

    print(f"Adjusted RC Overrides -> Roll: {roll_pwm}, Pitch: {pitch_pwm}")

def msg_receiver():
    time_last = 0
    time_to_wait = 0.1

    while True:
        message = cap.read()
        if time.time() - time_last > time_to_wait:
            np_data = np.array(message)
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
            (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
            cv2.circle(np_data, (horizontal_res // 2, vertical_res // 2), 8, (0, 0, 255), cv2.FILLED)

            if ids is not None and ids[0][0] == id_to_find:
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

                x_sum = sum([corner[0] for corner in corners[0][0]])
                y_sum = sum([corner[1] for corner in corners[0][0]])
                cx = x_sum / 4
                cy = y_sum / 4

                x_error = cx - (horizontal_res / 2)
                y_error = cy - (vertical_res / 2)

                x_ang = x_error * (horizontal_fov / horizontal_res)
                y_ang = y_error * (vertical_fov / vertical_res)

                adjust_position_no_pid(x_ang, y_ang, tvec[2])

                aruco.drawDetectedMarkers(np_data, corners)
                aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)

            cv2.imshow("Frame", np_data)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            time_last = time.time()

def arm_and_takeoff_nogps(aTargetAltitude):
    if vehicle.system_status.state != "STANDBY":
        print("Vehicle is not ready for arming!")
        return False
    
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while True:
        current_altitude = vehicle.rangefinder.distance
        print("Hovering at %f meters." % current_altitude)
        if current_altitude >= aTargetAltitude - 0.1:
            print("Reached altitude!")
            break
        else:
            thrust = 0.6
        time.sleep(0.2)
        set_attitude(thrust=thrust)
    state.set_system_state("loiter")

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

def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    if yaw_angle is None:
        yaw_angle = vehicle.attitude.yaw
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        1,
        1,
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),
        0,
        0,
        math.radians(yaw_rate),
        thrust
    )
    vehicle.send_mavlink(msg)

def land():
    vehicle.mode = VehicleMode("LAND")

def control():
    if state.get_system_state() == "takeoff":
        arm_and_takeoff_nogps(0.5)
        state.set_system_state("loiter")

    if state.get_system_state() == "loiter":
        hover_thread = threading.Thread(target=hover)
        aruco_thread = threading.Thread(target=msg_receiver)
        
        hover_thread.start()
        aruco_thread.start()
        
        hover_thread.join()
        aruco_thread.join()

def main():
    control()

if __name__ == "__main__":
    main()

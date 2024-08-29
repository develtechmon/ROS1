#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

## Variables ##
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72  # ArUco ID default is 72
marker_size = 20  # CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)

found_count = 0
not_found_count = 0

calib_path = r"/home/jlukas/Desktop/My_Project/ROS1/ROS1_open_cv/src/open_cv/aruco/calibration_files_new_cam/"

np_camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
np_dist_coeff = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

## Functions ##
time_last = 0
time_to_wait = .1  # 100ms

width=640
height=480

def image_callback(img_msg):
    global not_found_count, found_count, time_last, time_to_wait, id_to_find
    
    bridge = CvBridge()
    message = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    if time.time() - time_last > time_to_wait:
        np_data = np.array(message)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        
        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
        
        cv2.circle(np_data, (width//2, height//2),8, (0,0,255), cv2.FILLED)

        print("Detected ID: =" + str(ids))
        
        try:
            if ids is not None and ids[0][0] == id_to_find:
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
                #adjust_position_no_pid(x_ang, y_ang, z)
                
                # To adjust the drone position hovering with PID controller, work but might need to tune the PID
                #adjust_position_my_pid(x_error,y_error,z)
                
                # Work in progress
                # To adjust the drone position hovering with PID controller using simple pid library, can't get fix position. 
                #adjust_position_simple_pid(x_ang, y_ang,z)
                
                found_count += 1    
            else:
                not_found_count += 1
                
        except Exception as e:
            print("Target not Found")
            print(e)
            not_found_count += 1
            
        # Show the Frame
        cv2.imshow("Frame", np_data)
        
        # Add waitKey to allow OpenCV to process the window events
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User exited')

        time_last = time.time()
                    
def main():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('camera/image', Image, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

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

calib_path = r"/home/jlukas/Desktop/My_Project/ROS1/ROS1_open_cv/src/open_cv/aruco/calibration_files/"

np_camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
np_dist_coeff = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

## Functions ##
time_last = 0
time_to_wait = .1  # 100ms

def image_callback(img_msg):
    global not_found_count, found_count, time_last, time_to_wait, id_to_find
    
    bridge = CvBridge()
    message = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    if time.time() - time_last > time_to_wait:
        np_data = np.array(message)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        
        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

        print("Detected ID: =" + str(ids))
        
        try:
            if ids is not None and ids[0][0] == id_to_find:
                print("Operated ID: " + str(ids[0][0])) 
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                x = '{:.2f}'.format(tvec[0])
                y = '{:.2f}'.format(tvec[1])
                z = '{:.2f}'.format(tvec[2])
                
                marker_position = f'MARKER POSITION: x={x} y={y} z={z}'
                aruco.drawDetectedMarkers(np_data, corners)
                aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)

                cv2.putText(np_data, marker_position, (10, 50), 0, .5, (255, 0, 0), thickness=2)
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

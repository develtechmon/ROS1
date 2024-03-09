#!/usr/bin/env python

'''
To run this script i suggest the following in Gazebo simulation

run roslaunch gazebo_ros iris_world.launch
sim_vehicle -v ArduCopter -f --console --map

in the sim_vehicle mode
1. wait for gps to lock at STABILIZE
2. change to LOITER mode
3. takeoff at loiter mode at `rc 3 1700` --> 3 is our channel 3
4. once reached at desire height then run `rc 3 1500`.
5. open `rqt` and observe the change at new `image\new` topic

'''
## Imports ##
import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp

## Variables ##
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72 ## arucoID
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

def msg_receiver(message):
    global not_found_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
        
        try:
            if ids is not None:
                if ids[0] == id_to_find:
                    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])
                    
                    y_sum = 0
                    
                    x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
                    y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]

                    
                    # This will give us centre point of the aruco marker in the frame itself
                    x_avg = x_sum*.25
                    y_avg = y_sum*.25
                    
                    # X-angle and Y-angle which used by mavlink message
                    x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
                    y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)

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
                    not_found_count=not_found_count + 1
            else:
                not_found_count=not_found_count+1

        except:
            print("Target not found")
            not_found_count = not_found_count + 1
        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    
    else:
        return None

def subscriber():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()

if __name__ == "__main__":
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
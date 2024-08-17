'''
Author : Lukas
Python Version : 3.9.2
OpenCv Version : opencv-contrib-python==4.4.0.46
Tested On : Rpi Zero 2 Machine  - 17-August-2024 (First test)
          : Raspbian GNU/Liux 11 (Bullseye 32 bit)

Please refer to rpi_opencv_installation guide in raspberry pi repository.
Then install using following command

sudo python3 -m pip uninstall opencv-contrib-python
sudo python3 -m pip install opencv-contrib-python==4.4.0.46
sudo python3 -m pip install imutils

'''
import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

import time
import os
import platform
import sys
import math

## Aruco Setup 
id_to_find=72
marker_size=20 #cm

realWorldEfficiency=.7 ##Iterations/second are slower when the drone is flying. This accounts for that
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)

found_count = 0
not_found_count = 0

## Old Open CV 4.2.0.32
parameters = cv2.aruco.DetectorParameters_create()

## New Open CV > 4.7
#prameters =  cv2.aruco.DetectorParameters()
#detector = cv.aruco.ArucoDetector(dictionary, parameters)

calib_path=r"/home/jlukas/Desktop/My_Project/ROS1/aruco_marker_detection/calibration_files/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

print(cameraMatrix)
print(cameraDistortion)

## FPS Calculations
time_last = 0
time_to_wait = .1 ## 100ms
found_count = 0
not_found_count = 0

## Camera Resolution
width=640
height=480

#cap = WebcamVideoStream(height=height, width=width, src=0).start()
cap = WebcamVideoStream(src=0).start()

while True:
    frame = cap.read()

    if time.time() - time_last > time_to_wait:
        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)

        ids =''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
        
        cv2.circle(frame_np, (width//2, height//2), 8 , (0,0,255), cv2.FILLED)
        
        if ids is not None:
            print("Found these IDs in the frame:")
            print(ids)

        try:
            if ids is not None:
                if ids[0][0] == id_to_find:
                    print("Operated ID: " + str(ids[0][0])) 
                    ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
                    rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]
                    x="{:.2f}".format(tvec[0])
                    y="{:.2f}".format(tvec[1])
                    z="{:.2f}".format(tvec[2])
                
                    # Calculate the center of the Aruco marker
                    #x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
                    #y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
                    
                    x_sum = sum([corner[0] for corner in corners[0][0]])
                    y_sum = sum([corner[1] for corner in corners[0][0]])
                    
                    cx = x_sum / 4
                    cy = y_sum / 4
                    
                    # Calculate the error between the marker's center and the image center
                    x_error = cx - (horizontal_res / 2)
                    y_error = cy - (vertical_res / 2)
                    
                    # Convert these errors into angles for drone adjustment
                    # x_ang = (x_avg - horizontal_res*0.5)*(horizontal_fov/horizontal_res)
                    # y_ang = (y_avg - vertical_res*0.5)*(vertical_fov/vertical_res)
                    
                    x_ang = x_error * (horizontal_fov / horizontal_res)
                    y_ang = y_error * (vertical_fov / vertical_res)
                    
                    marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z
                    
                    aruco.drawDetectedMarkers(frame_np, corners)
                    aruco.drawAxis(frame_np, cameraMatrix, cameraDistortion, rvec, tvec,10)
                    
                    cv2.putText(frame_np, marker_position, (10,50),0,.5,(255,0,0), thickness=2)
                    
                    print(f"X CENTER PIXEL: {cx} Y CENTER PIXEL: {cy}")
                    print(f"FOUND COUNT: {found_count} NOTFOUND COUNT: {not_found_count}")
                    print(f"MARKER POSITION: x={x} y={y} z={z}\n")
                    print(f"ERROR POSITION: Error X={x_error} Error Y={y_error}\n")
                    
                    found_count += 1
                else:
                    not_found_count += 1
            else:
                not_found_count += 1                
            
        except Exception as e:
            print("Target not found")
            print(e)
            not_found_count += 1
        time_last = time.time()
        
    cv2.imshow("Frame",frame_np)
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break



    

    

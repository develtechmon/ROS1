'''
Author : Lukas
This code is tested on Windows with following spec:

Python Version : 3.8.10
OpenCv Version : opencv-contrib-python==4.2.0.32
Tested On : Windows Machine  - 17-August-2024 (First test)

Install Using Below Command
py -3.8 -m pip install  opencv-contrib-python==4.2.0.32
py -3.8 -m pip install imutils

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

width=640
height=480

cap = WebcamVideoStream(height=height, width=width, src=0).start()

while True:
    frame = cap.read()
    
    cv2.imshow("Frame",frame)
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break
    
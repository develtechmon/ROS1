## Getting Started

Please read. This script depend on `ros_numpy`. You have to build it from source to make it work.

###  Install ros_numpy from source
```
# Navigate to your ROS workspace
cd ~/My_Project/ROS1/aruco_drone/src

# Clone ros_numpy
git clone https://github.com/eric-wieser/ros_numpy.git

# Build the workspace
cd ~/My_Project/ROS1/aruco_drone
catkin_make

# Source the workspace
source devel/setup.bash
```

Then run 
```
rosrun aruco_drone keyboard_motion_control_sim_with_aruco_detection.py 
```

Secondly, this version work dedicated `openCv` version. You have to remove the current version and install this version
```
Step 1: Uninstall Current OpenCV
pip3 uninstall opencv-python opencv-contrib-python

Step 2: Install OpenCV 4.2.0
pip3 install opencv-python==4.2.0.34 opencv-contrib-python==4.2.0.34

Step 3: Verify Installation
python3 -c "import cv2; print(cv2.__version__)"
```

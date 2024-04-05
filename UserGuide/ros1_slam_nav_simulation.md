# Getting started

This is simulation `Guide` on how to start `SLAM` and `NAV` simulation in ROS1.
In this project, i'm using packages from `Robotics_ws`.

My machine setup as follow
```
Ubuntu 18 : Bionic
ROS1 : melodic
```

## Step 1: Install below packages
```
sudo apt install ros-melodic-gmapping
sudo apt-get install ros-melodic-dwa-local-planner
sudo apt-get install ros-melodic-teleop-twist-keyboard
sudo apt-get install ros-melodic-joy 
sudo apt-get install ros-melodic-teleop-twist-joy
sudo apt-get install ros-melodic-teleop-twist-keyboard
sudo apt-get install ros-melodic-laser-proc
sudo apt-get install ros-melodic-rgbd-launch
sudo apt-get install ros-melodic-depthimage-to-laserscan
sudo apt-get install ros-melodic-rosserial-Arduino
sudo apt-get install ros-melodic-rosserial-python
sudo apt-get install ros-melodic-rosserial-server
sudo apt-get install ros-melodic-rosserial-client
sudo apt-get install ros-melodic-rosserial-msgs
sudo apt-get install ros-melodic-amcl
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-move-base
sudo apt-get install ros-melodic-urdf
sudo apt-get install ros-melodic-xacro
sudo apt-get install ros-melodic-compressed-image-transport
sudo apt-get install ros-melodic-rqt-image-view
sudo apt-get install ros-melodic-rqt*
sudo apt-get install ros-melodic-gmapping
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-interactive-markers
sudo apt-get install ros-melodic-gazebo-ros-pkgs
sudo apt-get install gparted

New
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
```

## Step 2 : Clone and Setup the project

This package is based on original developer : `https://github.com/harshmittal2210/Robotics_ws.git`

I've made some minor changes to this original packages, you'll need to use my package to work in my environment as follow.
```
git clone https://github.com/develtechmon/ROS1.git
cd ROS1/SLAM_NAV
cd Robotics_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src (you can ignore the opencv if you see this message)
catkin_make
source devel/setup.sh
```

## Step 3 : Run our `Gazebo` Robot
```
roslaunch atom world.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/atom/cmd_vel
```

## Step 4 : Start our `Mapping`
```
roslaunch atom gmapping_demo.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/atom/cmd_vel
rosrun map_server map_saver -f map
```

## Step 5 : Run `Localization`
```
roslaunch atom localization.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/atom/cmd_vel
```

## Step 6 : Run `Navigation`
```
roslaunch atom navigation.launch
```










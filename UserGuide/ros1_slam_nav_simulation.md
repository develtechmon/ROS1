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
sudo apt-get install ros-melodic-gmapping
sudo apt-get install ros-melodic-navigation
```

## Step 2 : Clone and Setup the project
```
https://github.com/harshmittal2210/Robotics_ws.git
cd Robotics_ws
git submodule update --init --recursive
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
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











mkdir -p catkin_ws/src
cd catkin_ws/src

git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

if you have CATKIN_IGNORE file. Please move this file to urc-code

Then compile
cd catkin_ws
catkin_make

To launch our robot

roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

To increase the performance
gz physics -s 0.001 This allow the robot to get faster and faster

The large we make the number, the unstable the environment can get. For example
robot will shaking 

To control robot using keyboard
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Then add below line into .cshrc file
export TURTLEBOT3_MODEL=burger

env | grep "TURTLE"
This should return all matches containing "TURTLE"

or run 
echo $TURTLEBOT3_MODEL

Then add below line into .cshrc file
export TURTLEBOT3_MODEL=waffle_pi
echo $TURTLEBOT3_MODEL

source ~jlukas/.bashrc

Run this command to launch our world
roslaunch turtlebot3_gazebo turtlebot3_world.launch

To setup gz physics
gz physics -s 0.01 modifying this file will manipulate our robot.
Whether it's much more faster than real time

Increasing 0.01 value to higher will change the real time factor. 
Higher value will incraease real time factor

gz physics -s 0.015

To run mapping. This will launch gazebo and rviz
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

To control robot using keyboard for mapping
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

By default the map has been saved into this example..

Gaz physics
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:$HOME/map.yaml

This will launch rviz. You may need to adjust fi

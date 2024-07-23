# Getting Started

This is userguide on how to start simulation using `MAVROS`. This package, assume that you've
install mavros and ros1 melodic in your machine. If not, please refer to ros1_melodic_ubuntu_18_Gazebo_STL userguide. Please dollow the order by Sequence

## Run Gazebo

Open terminator terminal and run following command
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world 
```

Your gazebo should appear soon.

## Run SITL

In other terminal, please run SITL using following command based on below step
```
cd ~/ardupilot/ArduCopter

sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

This will launch SITL console into the windows

## Run APM Launcher

Since i'm running using MAVROS, it's important to run this launcher file first. Here,i'm going 
to show how you can create your own launch or launch directly from ROS

To run using default ros use following command. 
Here, tcp port is the same we use in our `dronekit` connection string
```
roslaunch mavros apm.launch fcu_url:=tcp://127.0.0.1:5763
```

To run using your own customized launch file, follow below method. By mean, we have to create our 
own packakge.

```
mkdir -p mavros_tutorial/src
cd src
catkin_create_pkg drone rospy roscpp std_msgs geometry_msgs

cd ..
catkin_make

cd src/drone
mkdir scripts
mkdir launcher

cd launcher

roscd mavros ---> To go to mavros directory
roscp mavros apm.launch apm.launch
chmod +x apm.launch

gedit apm.launch
```
This will launch gedit terminal. In the terminal, please change `fcu_url` to following

```
<launch>
	<!-- vim: set ft=xml noet : -->
	<!-- example launch script for ArduPilot based FCU's -->

	<!-- <arg name="fcu_url" default="/dev/ttyACM0:57600" /> -->
	<arg name="fcu_url" default="tcp://127.0.0.1:5763" /> <-------- This one
	<arg name="gcs_url" default="" />
	<arg name="tgt_system" default="1" />
	<arg name="tgt_component" default="1" />
	<arg name="log_output" default="screen" />
	<arg name="fcu_protocol" default="v2.0" />
	<arg name="respawn_mavros" default="false" />

	<include file="$(find mavros)/launch/node.launch">
		<arg name="pluginlists_yaml" value="$(find mavros)/launch/apm_pluginlists.yaml" />
		<arg name="config_yaml" value="$(find mavros)/launch/apm_config.yaml" />
		<arg name="fcu_url" value="$(arg fcu_url)" />
		<arg name="gcs_url" value="$(arg gcs_url)" />
		<arg name="tgt_system" value="$(arg tgt_system)" />
		<arg name="tgt_component" value="$(arg tgt_component)" />
		<arg name="log_output" value="$(arg log_output)" />
		<arg name="fcu_protocol" value="$(arg fcu_protocol)" />
		<arg name="respawn_mavros" value="$(arg respawn_mavros)" />
	</include>
</launch>
```

Save and quit.

And next, you have to `make again` as follow
```
cd ~mavros_tutorial
catkin_make
source devel/setup.bash

```

To launch the `apm launcher`, use following command. 
```
roslaunch drone apm.launch
```

This will launch the result as follow to indicate there is flight controller SIM detected.
```
[ INFO] [1721749328.519058709]: VER: 1.1: Flight software:     04060000 (87435473)
[ INFO] [1721749328.519084349]: VER: 1.1: Middleware software: 00000000 (        )
[ INFO] [1721749328.519124740]: VER: 1.1: OS software:         00000000 (        )
[ INFO] [1721749328.519148212]: VER: 1.1: Board hardware:      00000000
[ INFO] [1721749328.519167996]: VER: 1.1: VID/PID:             0000:0000
[ INFO] [1721749328.519182472]: VER: 1.1: UID:                 0000000000000000
[ INFO] [1721749337.517489174]: FCU: ArduCopter V4.6.0-dev (87435473)
[ INFO] [1721749337.517654407]: FCU: b4385e815bab498794cf9f418b75c6c7
[ INFO] [1721749337.517769636]: FCU: Frame: QUAD/X
[ INFO] [1721749338.177170781]: PR: parameters list received
[ INFO] [1721749342.516105663]: GF: mission received
[ INFO] [1721749342.516938426]: RP: mission received
[ INFO] [1721749342.517227089]: WP: mission received
```

## Run our python script 

In same working space directory, go to the following
```
cd src/drone/scripts

touch takeoff_land.py
chmod +x takeoff_land.py

nedit takeoff_land.py
```

and copy the following scripts. In general, this scripts will
perform takeoff and land.
```
#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)

        self.current_state = State()
        self.current_pose = PoseStamped()

        # Subscribe to the MAVROS state and local position topics
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Create services for arming and setting mode
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Create a publisher for the local position goal
        self.pose_goal_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    def state_callback(self, state):
        self.current_state = state

    def pose_callback(self, pose):
        self.current_pose = pose

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.set_mode('GUIDED')
            self.arming_client(True)
            rospy.loginfo("Drone armed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            self.set_mode_client(0, mode)
            rospy.loginfo("Mode set to %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def takeoff(self):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff(altitude=10.0)  # Set the desired takeoff altitude
            rospy.loginfo("Takeoff command sent. Response: %s", response)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def land(self):
        # Set the mode to LAND
        self.set_mode('LAND')
        rospy.loginfo("Landing...")

if __name__ == "__main__":
    try:
        controller = DroneController()

        # Arm the drone
        controller.arm()

        # Change the mode to GUIDED
        controller.set_mode('GUIDED')

        # Take off
        controller.takeoff()

        # Wait for 10 seconds
        rospy.loginfo("Waiting for 10 seconds...")
        time.sleep(10.0)

        # Land the drone
        controller.land()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

```

save and quit the terminal.

Next we're going to catkin make again. Use following approach
```
cd ~mavros_tutorial
catkin_make

source devel/setup.bash
```

To run our script python, use following command
```
rosrun drone takeoff_land.py
```
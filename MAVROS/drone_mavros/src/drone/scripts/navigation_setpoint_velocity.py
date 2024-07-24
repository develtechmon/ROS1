#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped
import time

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)

        self.current_state = State()

        # Subscribe to the MAVROS state topic
        rospy.Subscriber('/mavros/state', State, self.state_callback)

        # Create services for arming and setting mode
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Create a publisher for the velocity command
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    def state_callback(self, state):
        self.current_state = state

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
            response = takeoff(altitude=1)  # Set the desired takeoff altitude
            rospy.loginfo("Takeoff command sent. Response: %s", response)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def land(self):
        # Set the mode to LAND
        self.set_mode('LAND')
        rospy.loginfo("Landing...")

    def move(self, linear_x, linear_y, linear_z, angular_z):
        # linear_x, linear_y, linear_z: Linear velocities in m/s
        # angular_z: Angular velocity around z-axis in rad/s
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = rospy.Time.now()
        vel_cmd.twist.linear.x = linear_x
        vel_cmd.twist.linear.y = linear_y
        vel_cmd.twist.linear.z = linear_z
        vel_cmd.twist.angular.z = angular_z

        # Publish the velocity command
        self.velocity_pub.publish(vel_cmd)

    def stop(self):
        # Send zero velocity command to stop the drone
        self.move(0, 0, 0, 0)

if __name__ == "__main__":
    try:
        controller = DroneController()

        # Arm the drone
        controller.arm()

        # Change the mode to GUIDED
        controller.set_mode('GUIDED')

        # Take off
        controller.takeoff()
        time.sleep(5)

        # Move left, right, front, and back
        rospy.loginfo("Moving left...")
        controller.move(0.5, 0.0, 0.0, 0.0)  # Move left (1 m/s in x-direction)
        time.sleep(5.0)

        rospy.loginfo("Moving right...")
        controller.move(-0.5, 0.0, 0.0, 0.0)  # Move right (-1 m/s in x-direction)
        time.sleep(5.0)

        rospy.loginfo("Moving back...")
        controller.move(0.0, 0.5, 0.0, 0.0)  # Move front (1 m/s in y-direction)
        time.sleep(5.0)

        rospy.loginfo("Moving front...")
        controller.move(0.0, -0.5, 0.0, 0.0)  # Move back (-1 m/s in y-direction)
        time.sleep(5.0)

        # Stop the drone
        controller.stop()

        rospy.loginfo("Yaw left...")
        controller.move(0.0, 0.0, 0.0, 0.5)  # Move back (-1 m/s in y-direction)
        time.sleep(5.0)

        rospy.loginfo("Yaw right...")
        controller.move(0.0, 0.0, 0.0, -0.5)  # Move back (-1 m/s in y-direction)
        time.sleep(5.0)

        # Land the drone
        controller.land()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

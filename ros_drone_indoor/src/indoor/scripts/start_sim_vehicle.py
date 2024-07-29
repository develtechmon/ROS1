#!/usr/bin/env python

import rospy
import subprocess

def start_sim_vehicle():
    rospy.init_node('start_sim_vehicle')
    rospy.loginfo("Starting sim_vehicle.py")
    subprocess.Popen(['sim_vehicle.py', '-v', 'ArduCopter', '-f', 'gazebo-iris', '--console'])
    rospy.spin()

if __name__ == '__main__':
    try:
        start_sim_vehicle()
    except rospy.ROSInterruptException:
        pass


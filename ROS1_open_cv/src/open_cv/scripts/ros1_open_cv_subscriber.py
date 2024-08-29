#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # Display the image
        cv2.imshow("Camera Image", cv_img)
        cv2.waitKey(1)  # Display the image window
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", str(e))

def main():
    # Initialize the ROS node
    rospy.init_node('camera_subscriber', anonymous=True)
    
    # Create a subscriber for the camera image topic
    rospy.Subscriber('camera/image', Image, callback)
    
    # Spin to keep the node active
    rospy.spin()
    
    # Destroy all OpenCV windows when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

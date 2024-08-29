#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(img_msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('camera/image', Image, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

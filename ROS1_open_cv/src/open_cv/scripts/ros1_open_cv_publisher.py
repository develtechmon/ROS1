#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Create a publisher for the camera image topic
    image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
    
    # Create a CvBridge object to convert OpenCV images to ROS Image messages
    bridge = CvBridge()
    
    # Open the camera (0 is the default camera)
    cap = cv2.VideoCapture(0)
    
    # Check if the camera opened successfully
    if not cap.isOpened():
        rospy.logerr("Error: Camera not found.")
        return
    
    # Set the rate of publishing
    rate = rospy.Rate(15)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret:
            try:
                # Convert the frame to a ROS Image message
                ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                # Publish the image
                image_pub.publish(ros_img)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: %s", str(e))
        else:
            rospy.logwarn("Failed to capture image from camera.")
        
        # Sleep to maintain the desired publish rate
        rate.sleep()
    
    # Release the camera when done
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread, Lock

class WebcamVideoStream:
    def __init__(self, src=0, name="WebcamVideoStream"):
        self.stream = cv2.VideoCapture(src)
        
        # Reduce buffer size
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Set a lower resolution for better performance
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.name = name
        self.stopped = False
        self.frame = None
        self.lock = Lock()  # Thread safety lock

    def start(self):
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return
            grabbed, frame = self.stream.read()
            if grabbed:
                with self.lock:
                    self.frame = frame

    def read(self):
        with self.lock:
            return self.frame

    def stop(self):
        self.stopped = True

def publisher_thread(image_pub, bridge, cap, rate):
    while not rospy.is_shutdown():
        frame = cap.read()
        if frame is not None:
            try:
                # Convert OpenCV image to ROS Image message
                ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                image_pub.publish(ros_img)
            except CvBridgeError as e:
                rospy.logerr(e)
        rate.sleep()

def main():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
    bridge = CvBridge()
    cap = WebcamVideoStream(src=0).start()

    # Adjust rate to match camera FPS if possible
    rate = rospy.Rate(20)  # Adjust this based on the camera's FPS

    # Start the publisher in a separate thread to avoid blocking the main loop
    publisher_thread_handle = Thread(target=publisher_thread, args=(image_pub, bridge, cap, rate))
    publisher_thread_handle.start()

    rospy.spin()  # Keep the node running

    cap.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

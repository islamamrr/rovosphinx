#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

bridge = CvBridge()

ros_frame = 0
frame = 0

def image_callback(ros_image):
    global ros_frame
    global frame
    global bridge

    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    print frame.shape
    ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")

if __name__ == '__main__':

    rospy.init_node('sfwfds', anonymous=True)

    rate = rospy.Rate(10)

    image_sub = rospy.Subscriber("/rospicam/image/compressed",Image, image_callback)
    while not rospy.is_shutdown():
        if ros_frame != 0:
            rate.sleep()


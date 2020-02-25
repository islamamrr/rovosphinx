#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import pipes_func
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

    frame = pipes_func.yarab(frame)
    ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")

if __name__ == '__main__':

    rospy.init_node('rabbaaah', anonymous=True)

    state_publisher = rospy.Publisher("/coral_reefs", Image, queue_size=10)

    rate = rospy.Rate(10)

    image_sub = rospy.Subscriber("/cx",Image, image_callback)
    while not rospy.is_shutdown():
        if ros_frame != 0:
            state_publisher.publish(ros_frame)
            rate.sleep()


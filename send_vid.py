#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np

bridge = CvBridge()


if __name__ == '__main__':
    try:
        cap = cv2.VideoCapture(0)
        # hh = cv2.imread('xpipes.png')

        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(60)
        pub = rospy.Publisher('/cx', Image, queue_size=10)
                
        while not rospy.is_shutdown():
            ret, frame = cap.read()

            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
            
            pub.publish(image_message)
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
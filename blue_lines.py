#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ros_essentials_cpp.msg import TwoCentres

bridge = CvBridge()

lower_blue = np.array([110,80,30])
upper_blue = np.array([135,255,255])

kernel1 = np.ones((5,5), np.uint8)
kernel2 = np.ones((15,15), np.uint8)

frame = 0
ros_frame = 0
rflag = lflag = 0
centres = TwoCentres()

def image_callback(ros_image):

    global ros_frame
    global frame
    global bridge
    global lower_blue, upper_blue, kernel1, kernel2
    global rflag, lflag
    #convert ros_image into an opencv-compatible image
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)

    frame = cv2.flip(frame, 1)
    height,width = frame.shape[:2]
    
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_img, lower_blue, upper_blue)

    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel2)

    # Extract Contours
    _, contours, hierarchy = cv2.findContours(closing, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # print 'number of contours' + str(len(sorted_contours))
    i = 0
    for cnt in sorted_contours:
        if i == 2:
            break
        # Get approximate polygons
        hull = cv2.convexHull(cnt)

        cv2.drawContours(frame, [hull], 0, (0, 255, 0), 2)

        M = cv2.moments(hull)
        if int( M['m00']) != 0:
            if i == 0:
                cx_r = int(M['m10'] / M['m00'])
                cy_r = int(M['m01'] / M['m00'])
            else:
                cx_l = int(M['m10'] / M['m00'])
                cy_l = int(M['m01'] / M['m00'])

        i = i + 1
    
    try:
        cx_r, cx_l
    except NameError:
        pass        
    else:
        if cx_l > cx_r:
            temp = cx_l
            cx_l = cx_r
            cx_r = temp

            temp = cy_l
            cy_l = cy_r
            cy_r = temp

        # print "cx_r " + str(cx_r)
        # print "cx_l " + str(cx_l)

        centres.cx_r = cx_r
        centres.cx_l = cx_l

        centres_pub.publish(centres)
    
        right_margin = width - cx_r
        left_margin = cx_l

        if right_margin < 20:
            rflag = 1
        if rflag == 1:
            right_margin = 20
        if width - cx_r > 20 and width - cx_r < 50:
            rflag = 0

        if left_margin < 20:
            lflag = 1
        if lflag == 1:
            left_margin = 20
        if cx_l > 20 and cx_l < 50:
            lflag = 0

        error = right_margin - left_margin
        print 'right: ' + str(right_margin)
        print 'error' + str(error)
        print 'left: ' + str(left_margin)
        print '*********'

    ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")


if __name__ == '__main__':

    rospy.init_node('cv_ros', anonymous=True)

    state_publisher = rospy.Publisher("/blue_lines", Image, queue_size=10)
    centres_pub = rospy.Publisher("/centres", TwoCentres, queue_size=10)

    rate = rospy.Rate(60)

    image_sub = rospy.Subscriber("/cx",Image, image_callback)
    while not rospy.is_shutdown():
        if ros_frame != 0:
            state_publisher.publish(ros_frame)
            rate.sleep()

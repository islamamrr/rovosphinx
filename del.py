import cv2
import numpy as np

cap = cv2.VideoCapture(0)

lower_blue = np.array([110,60,50])
upper_blue = np.array([135,255,255])

kernel1 = np.ones((5,5), np.uint8)
kernel2 = np.ones((15,15), np.uint8)

rflag = lflag = 0

while(1):
    ret, frame = cap.read()

    frame = cv2.flip(frame, 1)
    height,width = frame.shape[:2]

    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
    cv2.imshow('mask', mask)

    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel2)

    # Extract Contours
    _, contours, hierarchy = cv2.findContours(closing, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    print 'number of contours' + str(len(sorted_contours))
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

        print "cx_r " + str(cx_r)
        print "cx_l " + str(cx_l)

        centres.cx_r = cx_r
        centres.cx_l = cx_l

        centres_pub.publish(centres)

        right_margin = width - cx_r
        left_margin = cx_l

        if right_margin < 20:
            rflag = 1
        if rflag == 1:
            right_margin = 20
        if width - cx_r > 20 and width - cx_r < 35:
            rflag = 0

        if left_margin < 20:
            lflag = 1
        if lflag == 1:
            left_margin = 20
        if cx_l > 20 and cx_l < 35:
            lflag = 0

        # error = right_margin - left_margin
        # print '*****'
        print right_margin
        print left_margin
        # print '*****'
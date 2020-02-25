import cv2
import numpy as np
import math


def yarab(image):

    kernel_big = np.ones((5,5), np.uint8)
    kernel_small = np.ones((3,3), np.uint8)
    final_kernel = np.ones((7,7), np.uint8)

    pink_left = [512,0]
    pink_bot = [512,0]
    white_left = [512,0]
    white_bot = [512,0]

    pink_new_centres = []
    pink_new_contours = []
    pink_old_centres = []
    pink_old_contours = []
    white_new_centres = []
    white_new_contours = []
    white_old_centres = []
    white_old_contours = []

    lower_pink = np.array([145,80,100])
    upper_pink = np.array([180,255,255])

    lower_white = np.array([0,0,185])
    upper_white = np.array([180,70,255])

    white_ref = cv2.imread('w_ref.jpg',0)
    pink_ref = cv2.imread('p_ref.jpg',0)

    reference = cv2.imread('pipes.png')
    reference = cv2.resize(reference, (276, 209), interpolation = cv2.INTER_AREA)
    reference = cv2.copyMakeBorder(reference, top = 40, bottom = 0, left = 0, right = 0, borderType=cv2.BORDER_CONSTANT, value = [0, 0, 0])
    # cv2.imshow('Reference', reference)
    # image = cv2.imread('xpipes.png')

    image = cv2.resize(image, (276, 249), interpolation = cv2.INTER_AREA)
    height, width = image.shape[:2]

    # cv2.imshow('Frame', image)

    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    ##################

    mask = cv2.inRange(hsv_img, lower_white, upper_white)
    # cv2.imshow('white', mask)

    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
    # cv2.imshow('open white', opening)

    closing_white = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel_small)
    # cv2.imshow('close white', closing_white)

    _, contours, hierarchy = cv2.findContours(closing_white, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    k = 0

    if len(sorted_contours) > 1:
        for c in sorted_contours:
            accuracy = 0.03 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, accuracy, True)

            w_l = list(c[c[:, :, 0].argmin()][0])
            w_b = list(c[c[:, :, 1].argmax()][0])
        
            if w_l[0] < white_left[0]:
                white_left = w_l
            if w_b[1] > white_bot[1]:
                white_bot = w_b

            k = k+1
            if k == 3:
                break
    elif len(sorted_contours) == 1:
        accuracy = 0.03 * cv2.arcLength(sorted_contours[0], True)
        approx = cv2.approxPolyDP(sorted_contours[0], accuracy, True)

        white_left = list(sorted_contours[0][sorted_contours[0][:, :, 0].argmin()][0])
        white_bot = list(sorted_contours[0][sorted_contours[0][:, :, 1].argmax()][0])

    ##################

    mask = cv2.inRange(hsv_img, lower_pink, upper_pink)
    # cv2.imshow('pink', mask)

    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
    # cv2.imshow('open pink', opening)

    closing_pink = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel_small)
    # cv2.imshow('close pink', closing_pink)

    _, contours, hierarchy = cv2.findContours(closing_pink, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    j = 0
    if len(sorted_contours) > 1:
        for c in sorted_contours:
            accuracy = 0.03 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, accuracy, True)

            p_l = list(c[c[:, :, 0].argmin()][0])
            p_b = list(c[c[:, :, 1].argmax()][0])

            if p_l[0] < pink_left[0]:
                pink_left = p_l
            if p_b[1] > pink_bot[1]:
                pink_bot = p_b

            j = j+1
            if j == 4:
                break

    elif len(sorted_contours) == 1:
        accuracy = 0.03 * cv2.arcLength(sorted_contours[0], True)
        approx = cv2.approxPolyDP(sorted_contours[0], accuracy, True)

        pink_left = list(sorted_contours[0][sorted_contours[0][:, :, 0].argmin()][0])
        pink_bot = list(sorted_contours[0][sorted_contours[0][:, :, 1].argmax()][0])

    ##################

    if pink_left[0] < white_left[0]:
        extreme_left = pink_left[0]
    else:
        extreme_left = white_left[0]

    if pink_bot[1] > white_bot[1]:
        extreme_bot = pink_bot[1]
    else:
        extreme_bot = white_bot[1]

    T = np.float32([[1, 0, -extreme_left], [0, 1,height-extreme_bot]])

    pink_translation = cv2.warpAffine(closing_pink, T, (width, height))
    white_translation = cv2.warpAffine(closing_white, T, (width, height))

    # cv2.imshow('Pink_trans', pink_translation)
    # cv2.imshow('pink ref', pink_ref)
    # cv2.imshow('White_trans', white_translation)
    # cv2.imshow('white ref', white_ref)

    ################

    white_sub_bleach = cv2.subtract(white_translation, white_ref)

    white_sub_bleach = cv2.morphologyEx(white_sub_bleach, cv2.MORPH_OPEN, final_kernel)

    white_sub_bleach = cv2.cvtColor(white_sub_bleach, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(white_sub_bleach, (0,int(0.8*height)), (width, height), (0,0,0), -1)
    white_sub_bleach = cv2.cvtColor(white_sub_bleach, cv2.COLOR_BGR2GRAY)

    white_sub_bleach = cv2.GaussianBlur(white_sub_bleach, (3,3), 0)
    _,white_sub_bleach = cv2.threshold(white_sub_bleach, 200, 255, cv2.THRESH_BINARY)

    # cv2.imshow('White Subtract New', white_sub_bleach)

    #---------------#

    white_sub_miss = cv2.subtract(white_ref, white_translation)

    white_sub_miss = cv2.morphologyEx(white_sub_miss, cv2.MORPH_OPEN, final_kernel)

    white_sub_miss = cv2.cvtColor(white_sub_miss, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(white_sub_miss, (0,int(0.8*height)), (width, height), (0,0,0), -1)
    white_sub_miss = cv2.cvtColor(white_sub_miss, cv2.COLOR_BGR2GRAY)

    white_sub_miss = cv2.GaussianBlur(white_sub_miss, (3,3), 0)
    _,white_sub_miss = cv2.threshold(white_sub_miss, 200, 255, cv2.THRESH_BINARY)
    white_sub_miss = cv2.dilate(white_sub_miss, kernel_small, iterations = 1)

    # cv2.imshow('White Subtract Old', white_sub_miss)

    ################

    pink_sub_gro_rcv = cv2.subtract(pink_translation, pink_ref)
    pink_sub_gro_rcv = cv2.morphologyEx(pink_sub_gro_rcv, cv2.MORPH_OPEN, final_kernel)

    pink_sub_gro_rcv = cv2.cvtColor(pink_sub_gro_rcv, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(pink_sub_gro_rcv, (0,int(0.8*height)), (width, height), (0,0,0), -1)
    pink_sub_gro_rcv = cv2.cvtColor(pink_sub_gro_rcv, cv2.COLOR_BGR2GRAY)

    pink_sub_gro_rcv = cv2.GaussianBlur(pink_sub_gro_rcv, (3,3), 0)
    _,pink_sub_gro_rcv = cv2.threshold(pink_sub_gro_rcv, 200, 255, cv2.THRESH_BINARY)

    # cv2.imshow('Pink Subtract New', pink_sub_gro_rcv)

    #---------------#

    pink_sub_damage = cv2.subtract(pink_ref, pink_translation)
    pink_sub_damage = cv2.morphologyEx(pink_sub_damage, cv2.MORPH_OPEN, final_kernel)

    pink_sub_damage = cv2.cvtColor(pink_sub_damage, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(pink_sub_damage, (0,int(0.8*height)), (width, height), (0,0,0), -1)
    pink_sub_damage = cv2.cvtColor(pink_sub_damage, cv2.COLOR_BGR2GRAY)

    pink_sub_damage = cv2.GaussianBlur(pink_sub_damage, (3,3), 0)
    _,pink_sub_damage = cv2.threshold(pink_sub_damage, 200, 255, cv2.THRESH_BINARY)

    # cv2.imshow('Pink Subtract Old', pink_sub_damage)

    ################

    _, contours, hierarchy = cv2.findContours(white_sub_bleach, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for c in sorted_contours:
        # print cv2.contourArea(c)
        if cv2.contourArea(c) > 300:
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # print cx, cy
            white_new_centres.append([cx,cy])
            white_new_contours.append(c)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(image,(x+extreme_left,y+extreme_bot-height),(x+w+extreme_left,y+h+extreme_bot-height),(0,0,255),2)#RED

    #--------------#

    _, contours, hierarchy = cv2.findContours(white_sub_miss, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for c in sorted_contours:
        # print cv2.contourArea(c)
        if cv2.contourArea(c) > 300:
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # print 'old w'
            # print cx, cy
            white_old_centres.append([cx,cy])
            white_old_contours.append(c)
            # x,y,w,h = cv2.boundingRect(c)
            # cv2.rectangle(image,(x+extreme_left,y+extreme_bot-height),(x+w+extreme_left,y+h+extreme_bot-height),(0,0,255),2)

    ################

    _, contours, hierarchy = cv2.findContours(pink_sub_gro_rcv, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for c in sorted_contours:
        # print cv2.contourArea(c)
        if cv2.contourArea(c) > 300:

            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # print 'new p'
            # print cx, cy
            # print c
            pink_new_centres.append([cx,cy])
            pink_new_contours.append(c)
        
    #---------------#

    _, contours, hierarchy = cv2.findContours(pink_sub_damage, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for c in sorted_contours:
        # print cv2.contourArea(c)
        if cv2.contourArea(c) > 300:
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # print cx, cy
            # print c
            pink_old_centres.append([cx,cy])
            pink_old_contours.append(c)
        
    ################

    # CHECK FOR BLEACH RECOVERY or GROWTH #
    for p,item in enumerate(pink_new_centres):
        flag = 0
        for w,item in enumerate(white_old_centres):
            x = pink_new_centres[p][0] - white_old_centres[w][0]
            y = pink_new_centres[p][1] - white_old_centres[w][1]
            distance = math.sqrt(x*x + y*y)
            if distance <= 20:
                # RECOVERY #
                x,y,w,h = cv2.boundingRect(pink_new_contours[p])
                cv2.rectangle(image,(x+extreme_left,y+extreme_bot-height),(x+w+extreme_left,y+h+extreme_bot-height),(255,0,0),2)#BLUE
                flag = 1
            if flag == 0:
                # GROWTH
                x,y,w,h = cv2.boundingRect(pink_new_contours[p])
                cv2.rectangle(image,(x+extreme_left,y+extreme_bot-height),(x+w+extreme_left,y+h+extreme_bot-height),(0,255,0),2)#GREEN

    # CHECK FOR DAMAGE #
    for p,item in enumerate(pink_old_centres):
        for w,item in enumerate(white_new_centres):
            x = pink_old_centres[p][0] - white_new_centres[w][0]
            y = pink_old_centres[p][1] - white_new_centres[w][1]
            distance = math.sqrt(x*x + y*y)
            if distance > 20:
                # DAMAGE #
                x,y,w,h = cv2.boundingRect(pink_old_contours[p])
                cv2.rectangle(image,(x+extreme_left,y+extreme_bot-height),(x+w+extreme_left,y+h+extreme_bot-height),(0,255,255),2)#YELLOW

    # CHECK FOR DAMAGE #
    # for p,item in enumerate(pink_new_centres):
    #     for w,item in enumerate(white_old_centres):
    #         x = pink_new_centres[p][0] - white_old_centres[w][0]
    #         y = pink_new_centres[p][1] - white_old_centres[w][1]
    #         distance = math.sqrt(x*x + y*y)
    #         if distance > 20:
    #             # DAMAGE #
    #             x,y,w,h = cv2.boundingRect(white_old_contours[w])
    #             cv2.rectangle(image,(x+extreme_left,y+extreme_bot-height),(x+w+extreme_left,y+h+extreme_bot-height),(0,255,255),2)#YELLOW

    edges = cv2.imread('edges.jpg',0)
    image[edges > 100] = (0, 255, 0)
    image = cv2.flip(image, 1)
    return image
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # print cv2.countNonZero(pink_subtract)
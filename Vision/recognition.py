# Copyright 2025 by Mohamed Montasser.
# All rights reserved.
# This file is part of the Shape & Color Recognition System Project.

import cv2
import numpy as np

def nothing(x):
    # any operation
    pass


cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_COMPLEX

while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    # Range for upper range
    lower_red = np.array([170, 120, 70])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    # Generating the final mask to detect red color
    maskred = mask1 + mask2

    # green mask
    maskgreen = cv2.inRange(hsv, (36, 25, 25), (70, 255, 255))

    # bluemask
    lower_blue = np.array([78, 158, 124])
    upper_blue = np.array([138, 255, 255])
    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])

    maskblue = cv2.inRange(hsv, low_blue, high_blue)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 30])
    maskblack = cv2.inRange(hsv, lower_black, upper_black)


    kernel = np.ones((5, 5), np.uint8)
    maskred = cv2.erode(maskred, kernel)
    maskgreen = cv2.erode(maskgreen, kernel)
    maskblue = cv2.erode(maskblue, kernel)
    maskblack = cv2.erode(maskblack, kernel)


    # Contours detection
    if int(cv2.__version__[0]) > 3:
        # Opencv 4.x.x
        contoursred, _ = cv2.findContours(maskred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursgreen, _ = cv2.findContours(maskgreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursblue, _ = cv2.findContours(maskblue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursblack, _ = cv2.findContours(maskblack, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    else:
        # Opencv 3.x.x
        _, contoursred, _ = cv2.findContours(maskred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contoursgreen, _ = cv2.findContours(maskgreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contoursblue, _ = cv2.findContours(maskblue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contoursblack, _ = cv2.findContours(maskblue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contoursred:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 400:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 3:
                cv2.putText(frame, "Red Triangle", (x, y), font, 1, (0, 0, 0))
            elif len(approx) == 4:
                cv2.putText(frame, "Red Rectangle", (x, y), font, 1, (0, 0, 0))
            elif len(approx) > 8:
                cv2.putText(frame, "Red Circle", (x, y), font, 1, (0, 0, 0))

    for cnt in contoursgreen:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 400:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 3:
                cv2.putText(frame, "Green Triangle", (x, y), font, 1, (0, 0, 0))
            elif len(approx) == 4:
                cv2.putText(frame, "Green Rectangle", (x, y), font, 1, (0, 0, 0))
            elif len(approx) > 8:
                cv2.putText(frame, "Green Circle", (x, y), font, 1, (0, 0, 0))

    for cnt in contoursblue:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 400:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 3:
                cv2.putText(frame, "Blue Triangle", (x, y), font, 1, (0, 0, 0))
            elif len(approx) == 4:
                cv2.putText(frame, "Blue Rectangle", (x, y), font, 1, (0, 0, 0))
            elif len(approx) > 6:
                cv2.putText(frame, "Blue Circle", (x, y), font, 1, (0, 0, 0))


    for cnt in contoursblack:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 400:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 3:
                cv2.putText(frame, "Black Triangle", (x, y), font, 1, (180, 180, 180))
            elif len(approx) == 4:
                cv2.putText(frame, "Black Rectangle", (x, y), font, 1, (180, 180, 180))
            elif len(approx) > 8:
                cv2.putText(frame, "Black Circle", (x, y), font, 1, (180, 180, 180))

    cv2.imshow("Frame", frame)
   # NOT NEEDED, USED FOR TESTING.
   # cv2.imshow("Mask red", maskred)
   # cv2.imshow("Mask green", maskgreen)
   # cv2.imshow("Mask blue", maskblue)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()




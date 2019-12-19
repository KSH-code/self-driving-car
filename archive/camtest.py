#!/usr/bin/python

import cv2
import rospy
import time
import numpy as np
from sensor_msgs.msg import Image
import math
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])


def callback(img_data):
    global bridge
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')


def lane_slope(roi):
    lines = cv2.HoughLines(roi, 1, np.pi / 180, 75, None, 0, 0)

    left_x1 = 0
    left_x2 = 0
    left_y1 = 0
    left_y2 = 0
    right_x1 = 0
    right_x2 = 0
    right_y1 = 0
    right_y2 = 0
    left_count = 0
    right_count = 0

    if lines is not None:
        for line in lines:
            for rho, theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                dy = float(y2 - y1)
                dx = float(x2 - x1)
                if dy / dx <= -0.2:
                    left_x1 += x1
                    left_x2 += x2
                    left_y1 += y1
                    left_y2 += y2
                    left_count += 1
                elif dy / dx >= 0.2:
                    right_x1 += x1
                    right_x2 += x2
                    right_y1 += y1
                    right_y2 += y2
                    right_count += 1

    if left_count > 0:
        left_x1 = left_x1 // left_count
        left_x2 = left_x2 // left_count
        left_y1 = left_y1 // left_count
        left_y2 = left_y2 // left_count
        cv2.line(roi, (left_x1, left_y1), (left_x2,
                                           left_y2), (255, 255, 255), 3, cv2.LINE_AA)
    if right_count > 0:
        right_x1 = right_x1 // right_count
        right_x2 = right_x2 // right_count
        right_y1 = right_y1 // right_count
        right_y2 = right_y2 // right_count
        cv2.line(roi, (right_x1, right_y1), (right_x2,
                                             right_y2), (255, 255, 255), 3, cv2.LINE_AA)
    return 0


if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    time.sleep(1)
    while not rospy.is_shutdown():
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 70, 140)
        roi = canny[220:320, :]
        lane_slope(roi)
        cv2.imshow("full", roi)
        # cv2.imshow("left", l_aoi)
        # cv2.imshow("right", r_aoi)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()

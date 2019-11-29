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
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


def lane_slope(frame):
    roi = frame[v:v + scan_height, :]

    roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi = cv2.GaussianBlur(roi, (5, 5), 0)
    roi = cv2.Canny(roi, 50, 80)
    lines = cv2.HoughLines(roi, 1, np.pi / 180, 80, None, 0, 0)
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
                cv2.line(canny, (x1, y1), (x2, y2), (255, 255, 255), 8)
    return 0


if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    time.sleep(1)
    while not rospy.is_shutdown():
        lane_slope(cv_image)

        cv2.imshow("full", canny)
        # cv2.imshow("left", l_aoi)
        # cv2.imshow("right", r_aoi)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()

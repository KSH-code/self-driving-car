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
                if dx != 0:
                    if dy / dx <= -0.2:
                        cv2.line(roi, (x1, y1), (x2, y2), (255, 255, 255), 8)
                    if dy / dx >= 0.2:
                        cv2.line(roi, (x1, y1), (x2, y2), (255, 255, 255), 8)
    return 0


if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    time.sleep(1)
    while not rospy.is_shutdown():
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        canny = cv2.Canny(blur, 70, 140)
        roi = canny[230:330, :]
        lane_slope(roi)
        cv2.imshow("full", roi)
        # cv2.imshow("left", l_aoi)
        # cv2.imshow("right", r_aoi)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()

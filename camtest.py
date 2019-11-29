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


def lane_slope(aoi):
    lines = cv2.HoughLines(aoi, 1, np.pi/180, 95)
    if lines is not None:
        left_x = 0
        left_y = 0
        right_x = 0
        right_y = 0
        l_c = 0
        r_c = 0
        for line in lines:
            for rho, theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                dx = float(x2 - x1)
                dy = float(y2 - y1)
                if dx == 0:
                    continue
                if 0.2 > abs(dy/dx) or 0.6 < abs(dy/dx):
                    continue
                slope = dy / dx
                if slope < 0:
                    left_x += x2 - x1
                    left_y += y2 - y1
                    l_c += 1
                else:
                    right_x += x2 - x1
                    right_y += y2 - y1
                    r_c += 1
                cv2.line(canny, (x1, y1), (x2, y2), (255, 255, 255), 8)
        left_x = left_x / float(max(l_c, 1))
        left_y = left_y / float(max(l_c, 1))
        right_x = right_x / float(max(r_c, 1))
        right_y = right_y / float(max(r_c, 1))
        return left_y / float(max(left_x, 1)) + right_y / float(max(right_x, 1))
    return 0


if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    time.sleep(1)
    while not rospy.is_shutdown():
        gray = cv_image
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        canny = cv2.Canny(blur, 140, 200)
        canny = canny[250:400, :]
        print(-math.degrees(math.atan(lane_slope(canny))))

        cv2.imshow("full", canny)
        # cv2.imshow("left", l_aoi)
        # cv2.imshow("right", r_aoi)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()

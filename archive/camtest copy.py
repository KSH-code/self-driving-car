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

lower_red = np.array([-17, 100, 100])
upper_red = np.array([3, 255, 255])


def callback(img_data):
    global bridge
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')


if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    time.sleep(1)
    while not rospy.is_shutdown():
        frame = cv2.GaussianBlur(cv_image, (3, 3), 0)
        lower_red = np.array([150, 100, 50])
        upper_red = np.array([180, 255, 200])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow('zzz', mask_red)
        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()

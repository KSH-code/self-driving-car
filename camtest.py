#!/usr/bin/python

import cv2, rospy, time
import numpy as np
from sensor_msgs.msg import Image
import math
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def get_line_slope_intercept(line):
    for x1, y1, x2, y2 in line:
        if x2-x1 == 0:
            return float('inf'), 0
    slope = (y2-y1 )/(x2-x1)
    intercept = y1 - slope * x1
    return slope, intercept

def callback(img_data):
    global bridge
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    time.sleep(1)
    while not rospy.is_shutdown():
        aoi = cv_image[300:450,]

        gray = cv2.cvtColor(aoi, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        canny = cv2.Canny(blur, 70, 140)
        hough_lines = cv2.HoughLinesP(canny, 1, np.pi/180, 80, 100, 50, 150)
        if hough_lines is not None:
          for line in hough_lines:
            for x1,y1,x2,y2 in line:
                line_lengths = []
        lines = []
        line_lengths = []
        if hough_lines is not None:
            for line in hough_lines:
                slope, intercept = get_line_slope_intercept(line)

                if slope == float('inf'):
                    continue

                for x1, y1, x2, y2 in line:
                    line_len = np.sqrt((y2-y1)**2 + (x2-x1)**2)

                lines.append((slope, intercept))
                line_lengths.append(line_len)

            if len(line_lengths) > 0:
              avg = -(np.dot(line_lengths, lines)/np.sum(line_lengths))[1]
              print('angle: ', math.atan(avg))
        cv2.imshow("camera", canny)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()

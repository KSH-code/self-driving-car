import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import math


class LineDetector:

    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.optimize_image)
        self.video = cv2.VideoWriter(
            '/home/nvidia/Desktop/1112.avi', cv2.VideoWriter_fourcc(*'XVID'), 30.0, (640, 480))

    def optimize_image(self, data):
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(
            data, 'bgr8'), cv2.COLOR_BGR2HSV)
        self.video.write(gray)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.canny = cv2.Canny(blur, 50, 110)

    def total_angle(self):
        return -math.degrees(math.atan(self.lane_slope(self.canny[250:300, :])))

    def lane_slope(self, aoi):
        lines = cv2.HoughLines(aoi, 1, np.pi/180, 70)
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
            left_x = left_x / float(max(l_c, 1))
            left_y = left_y / float(max(l_c, 1))
            right_x = right_x / float(max(r_c, 1))
            right_y = right_y / float(max(r_c, 1))
            return left_y / float(max(left_x, 1)) + right_y / float(max(right_x, 1))

        return 0

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        pass

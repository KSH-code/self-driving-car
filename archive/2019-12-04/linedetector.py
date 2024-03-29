import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

image_width = 640
scan_width, scan_height = 300, 80
roi_vertical_pos = 220
row_begin = scan_height // 2 - 10
lmid, rmid = scan_width, image_width - scan_width


class LineDetector:

    def __init__(self, topic):
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.result = np.zeros(
            shape=(scan_height, image_width, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.left_right = [-40, 680]
        rospy.Subscriber(topic, Image, self.conv_image)
        self.angle = 0

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = roi_vertical_pos
        roi = self.cam_img[250:360, :]

        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi = cv2.GaussianBlur(roi, (7, 7), 0)
        roi = cv2.Canny(roi, 70, 140)
        lines = cv2.HoughLines(roi, 1, np.pi / 180, 85, None, 0, 0)

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

        slope_l = 0
        slope_r = 0
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

            slope_l = float(left_y2 - left_y1) / \
                max(float(left_x2 - left_x1), 1)
        if right_count > 0:
            right_x1 = right_x1 // right_count
            right_x2 = right_x2 // right_count
            right_y1 = right_y1 // right_count
            right_y2 = right_y2 // right_count
            slope_r = float(right_y2 - right_y1) / \
                max(float(right_x2 - right_x1), 1)
        print(slope_l, slope_r)
        self.angle = math.degrees(-math.atan(slope_l + slope_r))

    def get_left_right(self):
        return self.angle

    def show_image(self):
        pass

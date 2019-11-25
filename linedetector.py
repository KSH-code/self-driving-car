import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math

class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.image_width = 640
        self.image_height = 480
        self.aoi_height = self.image_height // 2

        self.cam_img = np.zeros(shape=(self.image_height, self.image_width, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.optimize_image)

    def optimize_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        aoi = self.cam_img[240:480,]

        gray = cv2.cvtColor(aoi, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        self.canny = cv2.Canny(blur, 70, 140)

    def get_lines_slope(self):
        hough_lines = cv2.HoughLinesP(self.canny, 1, np.pi/180, 80, 100, 50, 150)
        line_lengths = []
        sum_of_slope = 0
        if hough_lines is not None:
            for line in hough_lines:
                slope = self.get_line_slope(line)

                if slope == float('inf'):
                    continue

                sum_of_slope += slope
                line_lengths.append(1)

            avg = sum_of_slope / len(line_lengths) if len(line_lengths) > 0 else None
        else:
            avg = None
        return avg

    def get_line_slope(self, line):
        for x1, y1, x2, y2 in line:
            if x2-x1 == 0:
                return float('inf'), 0
        slope = (y2-y1 )/(x2-x1)
        return slope

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        pass

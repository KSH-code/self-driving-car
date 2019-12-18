import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

image_width = 640
scan_height = 100
row_begin = 50
lmid, rmid = 200, 440


class LineDetector:

    def __init__(self, topic):
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.result = np.zeros(
            shape=(scan_height, image_width, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.left_right = [-40, 680]
        rospy.Subscriber(topic, Image, self.conv_image)
        self.traffic_light_detected_count = 0

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.detect_line()
        self.detect_taffic_light()

    def detect_taffic_light(self):
        frame = cv2.GaussianBlur(self.cam_img, (3, 3), 0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1,
                                   100, param1=20, param2=50, minRadius=0, maxRadius=40)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                radius = i[2]

                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                pos = [center]
                visited_position = {center}
                red_pixel_count = 0
                while len(pos) > 0:
                    x, y = pos.pop(0)
                    h, s, v = hsv[y, x]
                    for dx, dy in zip([-1, 0, 1, 0], [0, 1, 0, -1]):
                        cur_x = x + dx
                        cur_y = y + dy
                        cur_pos = (cur_x, cur_y)
                        if 0 < cur_x < 640 and 0 < cur_y < 480 and cur_pos not in visited_position:
                            visited_position.add(cur_pos)
                            if 150 <= h <= 180 and 100 <= s <= 255 and 100 <= v <= 255:
                                pos.append(cur_pos)
                                red_pixel_count += 1
                                if red_pixel_count >= 200:
                                    self.left_right = False
                                    self.traffic_light_detected_count = 100
                                    break

    def detect_line(self):
        if self.traffic_light_detected_count > 0:
            print(self.traffic_light_detected_count)
            self.traffic_light_detected_count -= 1
            return
        self.left_right = [-40, 680]
        roi = self.cam_img[270:370, :]

        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi = cv2.GaussianBlur(roi, (5, 5), 0)
        roi = cv2.Canny(roi, 70, 140)
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

        result = np.zeros((scan_height, image_width, 3), dtype=np.uint8)
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
                    if dx == 0:
                        continue
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
        if right_count > 0:
            right_x1 = right_x1 // right_count
            right_x2 = right_x2 // right_count
            right_y1 = right_y1 // right_count
            right_y2 = right_y2 // right_count

        cv2.line(result, (left_x1, left_y1), (left_x2,
                                              left_y2), (255, 255, 255), 3, cv2.LINE_AA)
        cv2.line(result, (right_x1, right_y1), (right_x2,
                                                right_y2), (255, 255, 255), 3, cv2.LINE_AA)
        left, right = -40, 680
        for l in range(lmid):
            if np.all(result[row_begin, l] == [255, 255, 255]):
                left = l
                break
        for r in range(image_width - 1, rmid, -1):
            if np.all(result[row_begin, r] == [255, 255, 255]):
                right = r
                break
        if left != -40 or right != 680:
            self.left_right = [left, right]

    def get_left_right(self):
        return self.left_right

    def show_image(self):
        pass

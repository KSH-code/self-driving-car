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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1,
                                   100, param1=20, param2=60, minRadius=0, maxRadius=40)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                left = -1
                center = (i[0], i[1])
                radius = i[2]

                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                pos = [center]
                visited_position = {center}
                red_pixel_count = 0
                detection = False
                while len(pos) > 0:
                    x, y = pos.pop(0)
                    h, s, v = hsv[y, x]
                    for dx, dy in zip([-1, 0, 1, 0], [0, 1, 0, -1]):
                        cur_x = x + dx
                        cur_y = y + dy
                        cur_pos = (cur_x, cur_y)
                        if 0 < cur_x < 640 and 0 < cur_y < 480 and cur_pos not in visited_position:
                            visited_position.add(cur_pos)
                            cur_h, cur_s, cur_v = hsv[cur_y, cur_x]
                            print(cur_h, cur_s, cur_v)
                            if 150 <= h <= 180 and 100 <= s <= 255 and 100 <= v <= 255:
                                pos.append(cur_pos)
                                red_pixel_count += 1
                                if red_pixel_count >= 200:
                                    detection = True
                                    break
                cv2.imshow('zzz', frame)
        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()

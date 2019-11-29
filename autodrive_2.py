#!/usr/bin/env python

import rospy
import time
import math

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver


class AutoDrive:

    def __init__(self):
        rospy.init_node('autodrive')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        # self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        # obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        self.load_drive_info()
        self.driver.drive(self.angle + 90, self.speed + 90)

    def steer(self, left, right):
        angle = self.line_detector.total_angle()
        mid = (left + right) // 2 - 320
        if mid <= 30:
            angle = mid // 3
        else:
            angle = mid // 1.7
            angle = max(-50, angle) if angle < 0 else min(50, angle)
        return angle

    def exit(self):
        self.line_detector.video.release()
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)

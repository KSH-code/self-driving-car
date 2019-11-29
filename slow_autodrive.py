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
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        # obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        self.load_drive_info()
        self.driver.drive(self.angle + 90, self.speed + 90)

    def load_drive_info(self):
        angle = int(self.line_detector.total_angle())
        abs_angle = abs(angle)
        print(angle, abs_angle)
        if abs_angle <= 2:
            self.angle = 0.1
        elif abs_angle <= 10:
            self.angle = 0.2
        elif abs_angle <= 14:
            self.angle = 2.5
        else:
            self.angle = 20
        self.angle = int(self.angle * 25)
        if angle < 0:
            self.angle *= -1
        self.speed = 25

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)

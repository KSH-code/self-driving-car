#!/usr/bin/env python

import rospy
import time

from linedetector import LineDetector
from motordriver import MotorDriver
from obstacledetector import ObstacleDetector


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.obstacledetector = ObstacleDetector('/ultrasonic')
        self.s = 0

    def trace(self):
        left, right = self.line_detector.get_left_right()
        angle = self.steer(left, right)
        speed = self.accelerate(angle)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2 - 320
        if abs(mid) > 40:
            angle = mid // 1.5
        else:
            angle = mid // 3.5
        angle = max(-50, angle) if angle < 0 else min(50, angle)
        print(self.obstacledetector.get_distance())
        if self.obstacledetector.get_distance()[0] < 106 or self.s > 0:
            angle = 30
            if self.s > 0:
                self.s -= 1
            else:
                self.s = 5
        elif self.obstacledetector.get_distance()[2] < 106 or self.s < 0:
            angle = -30
            if self.s < 0:
                self.s += 1
            else:
                self.s = -5
        else:
            self.s = 0
        return angle

    def accelerate(self, angle):
        return 44

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

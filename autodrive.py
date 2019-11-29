#!/usr/bin/env python

import rospy
import time

from linedetector import LineDetector
from motordriver import MotorDriver


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        left, right = self.line_detector.get_left_right()
        angle = self.steer(left, right)
        speed = self.accelerate(angle)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2 - 320
        if mid <= 30:
            angle = mid // 3
        else:
            angle = mid // 1.7
            angle = max(-50, angle) if angle < 0 else min(50, angle)
        return angle

    def accelerate(self, angle):
        if angle <= -10 or angle >= 10:
            speed = 40
        else:
            speed = 50
        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
    rospy.on_shutdown(car.exit)

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
        result = self.line_detector.get_left_right()
        if result:
            if result == 'left':
                angle = 30
            elif result == 'right':
                angle = -30
            else:
                left, right = result
                angle = self.steer(left, right)
            speed = self.accelerate(angle)
        else:
            angle = 0
            speed = 0
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2 - 320
        if abs(mid) > 40:
            angle = mid // 1.5
        else:
            angle = mid // 3.5
        angle = max(-50, angle) if angle < 0 else min(50, angle)
        return angle

    def accelerate(self, angle):
        return 31

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

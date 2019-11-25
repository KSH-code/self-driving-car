#!/usr/bin/env python

import rospy, time
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
        self.angle = 0

    def trace(self):
        # obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        # line_l, line_r = self.line_detector.detect_lines()
        # self.line_detector.show_images(line_l, line_r)
        angle = self.steer()
        speed = self.accelerate()
        # self.driver.drive(angle + 90, speed + 90)

    def steer(self):
        slope = self.line_detector.get_lines_slope()

        print(self.angle)
        if slope is None:
            return self.angle
        else:
            print('slope', slope)
            print('angle', math.atan(slope))
            angle = math.atan(slope)
            if angle < 0:
                angle = max(-30, angle)
            else:
                angle = min(30, angle)
            self.angle = angle
            return angle

    def accelerate(self):
        return 20

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

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

    def trace(self):
        # obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        # line_l, line_r = self.line_detector.detect_lines()
        # self.line_detector.show_images(line_l, line_r)
        angle = self.steer()
        speed = self.accelerate()
        self.driver.drive(angle + 90, speed + 90)

    def steer(self):
        slope = self.line_detector.get_lines_slope_intecept()

        print(slope)
        if slope == None:
            return 0
        else:
            print(math.atan(slope))
            return math.atan(slope)

    def accelerate(self):
        return 30

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

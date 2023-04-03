#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(Image, 'camera/image', self.image_callback, 10)
        self.target_x = None
        self.target_y = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 48, 80])
        upper_color = np.array([20, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                self.target_x = int(M['m10'] / M['m00'])
                self.target_y = int(M['m01'] / M['m00'])
            else:
                self.target_x = None
                self.target_y = None

    def follow_target(self):
        if self.target_x is not None and self.target_y is not None:
            twist_msg = Twist()
            twist_msg.linear.x = 0.1  # 선속도 0.1 m/s
            twist_msg.angular.z = 0.001 * (self.target_x - 320) # 각속도 = (사람 위치 - 중심) * 0.001
            self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    while rclpy.ok():
        follower.follow_target()
        rclpy.spin_once(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


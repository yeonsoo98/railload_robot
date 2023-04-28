#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox

class Follower(Node):

    def __init__(self):
        super().__init__('follower_node')
        qos_profile = QoSProfile(depth=10)

        self.bbox_sub = self.create_subscription(
            BoundingBoxes, 'yolov5/bounding_boxes', self.bbox_callback, qos_profile)

        # 추가 cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
    
        self.target_box = [0, 0, 0, 0]
        
        # 추가 
        self.target_class = 'person'
        self.image_center = 640 // 2  # assuming image width of 640 pixels
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def bbox_callback(self, msg):
        for box in msg.bounding_boxes:
            self.get_logger().info(str(box.class_id) + str(box.id))
            self.target_box = [box.xmin, box.ymin, box.xmax, box.ymax]
            self.get_logger().info(str(self.target_box))
            
    def track_object(self):
        # 중심 계산 
        object_center = (self.target_box[2] + self.target_box[0]) / 2

        # 거래 계산
        distance = object_center - self.image_center

        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(twist_msg)
        
        # 각속도 관련
        # 각속도 필요 없을 듯
        # angular_velocity = -1 * distance / self.image_center * self.angular_spee
        # twist_msg.angular.z = angular_velocity
        

def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

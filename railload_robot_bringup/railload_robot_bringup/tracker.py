#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox

class Follower(Node):

    def __init__(self):
        super().__init__('follower_node')
        qos_profile = QoSProfile(depth=10)

        self.bbox_sub = self.create_subscription(
            BoundingBoxes, 'yolov5/bounding_boxes', self.bbox_callback, qos_profile)

        #self.target = -1
        self.target_box = [0, 0, 0, 0]

    def bbox_callback(self, msg):
        for box in msg.bounding_boxes:
            self.get_logger().info(str(box.class_id) + str(box.id))
            self.target_box = [box.xmin, box.ymin, box.xmax, box.ymax]
            self.get_logger().info(str(self.target_box))
            

def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

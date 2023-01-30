#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class Driving(Node):
    def __init__(self):
        super().__init__("driving_node")
        
        self.get_logger().info("init driving node ! ")



def main(args=None):
    rclpy.init(args=args)
    driving = Driving()
    rclpy.spin(driving)
    driving.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

class Driving(Node):
    def __init__(self):
        super().__init__("driving_node")
        
        self.get_logger().info("init driving node ! ")
        
        # robot move 
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10)


        self.odom_publisher
        
        # robot speed 
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        self.cmd_vel_publisher

        # response
    # robot cmd_vel callback
    def cmd_vel_callback(self, msg):
    
        self.info_msg.speed.linear.x = msg.linear.x
        self.info_msg.speed.linear.y = msg.linear.y
        self.info_msg.speed.angular.x = msg.angular.x     

    # robot odom callback
    def odom_callback(self, msg) :

        self.info_msg.position.x = msg.pose.pose.position.x
        self.info_msg.position.y = msg.pose.pose.position.y
        self.info_msg.position.z = msg.pose.pose.position.z        
        
def main(args=None):
    rclpy.init(args=args)
    driving = Driving()
    rclpy.spin(driving)
    driving.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
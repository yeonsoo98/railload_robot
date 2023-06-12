#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge

import numpy as np
import math

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox
from std_msgs.msg import Float32, Int8

class Follower(Node):

    def __init__(self):
        super().__init__('follower_node')
        qos_profile = QoSProfile(depth=10)
        self.bridge = CvBridge()

        # Subscriber
        self.bbox_sub = self.create_subscription(
            BoundingBoxes, 'yolov5/bounding_boxes', self.bbox_callback, qos_profile)
        self.depth_sub = self.create_subscription(
            Image, 'camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, 'camera/aligned_depth_to_color/camera_info', self.depth_info_callback, qos_profile)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.min_depth_pub = self.create_publisher(Float32, 'min_depth', qos_profile)
        self.count_people_pub = self.create_publisher(Int8, 'count_people', qos_profile)

        # Timer
        self.control_timer = self.create_timer(1, self.control_callback)

        # target related
        self.target_box = []
        self.target_class = 'person'
        self.target_depth = []
        self.target_flag = False

        # control related
        self.linear_speed = 0.5
        self.cmd_vel = Twist()

        # depth related
        self.m_fx = 0.0
        self.m_fy = 0.0
        self.depth_array = np.zeros(1)

    def bbox_callback(self, msg):

        for box in msg.bounding_boxes:
            if box.class_id == self.target_class:
                # self.get_logger().info(str(box.class_id) + str(box.id))
                self.target_box.append([box.xmin, box.ymin, box.xmax, box.ymax])
                # self.get_logger().info(str(self.target_box))
                self.target_flag = True
            else:
                self.target_box = []
                self.target_depth = []
                self.target_flag = False

    def depth_info_callback(self, msg):
        # Intrinsic camera matrix for the raw (distorted) images.
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        if msg.k[0] != 0.0:
            self.m_fx = msg.k[0];
            self.m_fy = msg.k[4];
            self.m_cx = msg.k[2];
            self.m_cy = msg.k[5];
            self.inv_fx = 1. / self.m_fx;
            self.inv_fy = 1. / self.m_fy;

    def depth_callback(self, msg):
        self.target_depth = []

        if self.target_flag:
            # ros2 Image msg to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')

            for box in self.target_box:
                x = box[0]
                y = box[1]
                w = box[2] - box[0]
                h = box[3] - box[1]

                # Boxes as large as 30px centered on the center of the box
                x = x + w//2 - 15
                y = y + h//2 - 15
                w = 30
                h = 30

                roi_depth = depth_image[y:y+h, x:x+w]

                inv_fx = 1. / self.m_fx
                inv_fy = 1. / self.m_fy

                n = 0
                sum = 0
                for i in range(0, roi_depth.shape[0]):
                    for j in range(0, roi_depth.shape[1]):
                        value = roi_depth.item(i, j)
                        # self.get_logger().info(str(value))
                        if value > 0.0:
                            n = n + 1
                            sum = sum + value
                try:
                    point_z = sum / n * 0.001
                    point_x = ((x + w/2) - self.m_cx) * point_z * inv_fx
                    point_y = ((y + h/2) - self.m_cy) * point_z * inv_fy
                    self.depth_array[0] = np.array(math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z))
                    mean_depth = float(self.depth_array.mean(axis=0))
                    self.target_depth.append(mean_depth)
                except Exception as e:
                    self.get_logger().error("Fail to calculate depth : {}".format(e))

        # Publish count_people
        try:
            count_people_msg = Int8()
            count_people_msg.data = len(self.target_depth)
            self.count_people_pub.publish(count_people_msg)
        except Exception as e:
            pass

    def control_callback(self):
        try:
            min_depth_msg = Float32()
            min_depth_msg.data = min(self.target_depth)
            self.get_logger().info('closest person distance' + str(round(min_depth_msg.data, 3)) + 'm')

            self.min_depth_pub.publish(min_depth_msg)

            # depth range: 0.5m ~ 1.5m
            if 0.5 < min_depth_msg.data < 1.5:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
            elif min_depth_msg.data < 0.5:
                self.cmd_vel.linear.x = -self.linear_speed
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
            elif min_depth_msg.data > 1.5:
                self.cmd_vel.linear.x = self.linear_speed
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)

        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

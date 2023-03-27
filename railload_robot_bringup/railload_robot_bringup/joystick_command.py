#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy

"""
buttons[0] : A
buttons[1] : B
buttons[2] : X
buttons[3] : Y
"""

class JoystickCommand(Node):
    def __init__(self):
        super().__init__("joystick_command_node")
        qos_profile = QoSProfile(depth=10)


        # publisher
        self.joystick_command = Int16()
        self.joystick_command_publisher = self.create_publisher(Int16,'joystick_command', qos_profile)

        # subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.get_logger().info("init Joystick Command node ! ")

    def joy_callback(self, msg):
        """
        Command ID
        0 => A + B
        1 => A + Y
        ...
        """

        if msg.buttons[0] and msg.buttons[1]:
            self.get_logger().info("joystick command 0")
            self.joystick_command.data = 0
            self.joystick_command_publisher.publish(self.joystick_command)

        elif msg.buttons[0] and msg.buttons[3]:
            self.get_logger().info("joystick command 1")
            self.joystick_command.data = 1
            self.joystick_command_publisher.publish(self.joystick_command)

def main(args=None):
    rclpy.init(args=args)
    joystick_command = JoystickCommand()
    
    try:
        rclpy.spin(joystick_command)
    except KeyboardInterrupt:
        
        joystick_command.get_logger().info("Keyboard interrupt, shutting down")
        joystick_command.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()

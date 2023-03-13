#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from rclpy.qos import QoSProfile
import termios
import tty
import sys

RPMBindings = {
    'q' : 300,
    "w" : 500,
    'e' : 700,
    'r' : 1000,
    's' : 0 ,
    'z' : -1000
}

msg = """
--------------
q => 300RPM
w => 500RPM
e => 700RPM
r => 1000RPM
--------------
s => stop
--------------
z => -1000RPM
--------------
"""

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardCommand(Node):
    def __init__(self):
        super().__init__("keyboard_command_node")
        qos_profile = QoSProfile(depth=10)

        # keyboard intterupt setting
        self.settings = termios.tcgetattr(sys.stdin)

        # keyboard input timer
        key_input_period = 0.1
        self.key_input_timer = self.create_timer(key_input_period, self.key_input_callback)

        # publisher
        self.input_rpm_publisher = self.create_publisher(Int16,'input_rpm', qos_profile)

        self.get_logger().info("init KeyboardCommand node ! ")

    # keyboard input timer
    def key_input_callback(self):
        self.get_logger().info(msg)
        key = getKey(self.settings)
        if key in RPMBindings.keys():
            self.key_input_rpm_publisher.publish(Int16(data=RPMBindings[key]))


def main(args=None):
    rclpy.init(args=args)
    keyboard_command = KeyboardCommand()
    rclpy.spin(keyboard_command)

    # Destroy the node explicitly
    keyboard_command.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

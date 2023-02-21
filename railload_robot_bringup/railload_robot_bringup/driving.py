#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile

import minimalmodbus as minimalmodbus
import serial
import time

ADDRESS = 1


COM_MODE =  [0x01, 0x06, 0x00, 0x17, 0x00, 0x01, 0xF8, 0x02]
SERVO_ON =  [0x01, 0x06, 0x00, 0x78, 0x00, 0x01, 0xC8, 0x13]
SERVO_OFF = [0x01, 0x06, 0x00, 0x78, 0x01, 0x00, 0x08, 0x43]
BREAK_ON =  [0x01, 0x06, 0x00, 0x78, 0x01, 0x01, 0xC9, 0x83]
BREAK_OFF = [0x01, 0x06, 0x00, 0x78, 0x01, 0x00, 0x08, 0x43]
RPM_P1000 = [0x01, 0x06, 0x00, 0x79, 0x03, 0xE8, 0x58, 0xAD]
RPM_0 =     [0x01, 0x06, 0x00, 0x79, 0x00, 0x00, 0x58, 0x13]
RPM_N1000 = [0x01, 0x06, 0x00, 0x79, 0xFC, 0x18, 0x19, 0x19]

class Driving(Node):
    def __init__(self):
        super().__init__("driving_node")
        qos_profile = QoSProfile(depth=10)

        # motor driver setup
        self.instrument = minimalmodbus.Instrument("/dev/motor_driver", ADDRESS)  # port name, slave address (in decimal)
        self.instrument.serial.baudrate = 115200  # Baud
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_NONE
        self.instrument.clear_buffers_before_each_transaction = True
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 0.05 # seconds
        self.instrument.mode = minimalmodbus.MODE_RTU  # rtu or ascii mode

        try:
            self.instrument.serial.write(bytes(COM_MODE))
            time.sleep(0.1)
            self.instrument.serial.write(bytes(SERVO_ON))
            time.sleep(0.1)
            self.instrument.serial.write(bytes(BREAK_OFF))
            time.sleep(0.1)
            self.instrument.serial.write(bytes(RPM_0))
        except Exception as e:
            self.get_logger().error("Fail to initialize : {}".format(e))
            self.destroy_node()
            rclpy.shutdown()

        # subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # publisher
        self.req_rpm_publisher = self.create_publisher(Int32, 'req_rpm', qos_profile)
        self.cur_rpm_publisher = self.create_publisher(Int32, 'cur_rpm', qos_profile)

        # timer
        timer_period= 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("init driving node ! ")


    # info timer
    def timer_callback(self):
        try:
            # publish current rpm
            cur_rpm = Int32()
            cur_rpm.data = self.instrument.read_register(registeraddress=0x03, number_of_decimals=0, functioncode=4, signed=False)
            self.cur_rpm_publisher.publish(cur_rpm)

            # publish required rpm
            req_rpm = Int32()
            req_rpm.data = self.instrument.read_register(registeraddress=0x02, number_of_decimals=0, functioncode=4, signed=False)
            self.req_rpm_publisher.publish(req_rpm)
        except Exception as e:
            self.get_logger().error("Fail to read : {}".format(e))

    # robot cmd_vel callback
    def cmd_vel_callback(self, msg):
        try:
            if msg.linear.x > 0.0:
                self.instrument.serial.write(bytes(RPM_P1000))
            elif msg.linear.x < 0.0:
                self.instrument.serial.write(bytes(RPM_N1000))
            else:
                self.instrument.serial.write(bytes(RPM_0))
        except Exception as e:
            self.get_logger().error("Fail to write : {}".format(e))

def main(args=None):
    rclpy.init(args=args)
    driving = Driving()
    rclpy.spin(driving)

    # Destroy the node explicitly
    driving.instrument.serial.close()
    driving.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
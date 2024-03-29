#!/usr/bin/env python3.8
# -*- coding: utf-8 -*- #

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Int16
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

RPM_P0030 = [0x01, 0x06, 0x00, 0x79, 0x00, 0x1E, 0xD8, 0x1B]
RPM_P0050 = [0x01, 0x06, 0x00, 0x79, 0x00, 0x32, 0xD9, 0xC6]
RPM_P0100 = [0x01, 0x06, 0x00, 0x79, 0x00, 0x64, 0x59, 0xF8]
RPM_P0200 = [0x01, 0x06, 0x00, 0x79, 0x00, 0xC8, 0x85, 0x59]
RPM_P0300 = [0x01, 0x06, 0x00, 0x79, 0x01, 0x2C, 0x58, 0x5E]
RPM_P0500 = [0x01, 0x06, 0x00, 0x79, 0x01, 0xF4, 0x58, 0x04]
RPM_P0700 = [0x01, 0x06, 0x00, 0x79, 0x02, 0xBC, 0x58, 0xC2]
RPM_P1000 = [0x01, 0x06, 0x00, 0x79, 0x03, 0xE8, 0x58, 0xAD]
RPM_P2000 = [0x01, 0x06, 0x00, 0x79, 0x07, 0xD0, 0x5B, 0xBF]
RPM_P3000 = [0x01, 0x06, 0x00, 0x79, 0x0B, 0xB8, 0x5F, 0x51]

RPM_0 =     [0x01, 0x06, 0x00, 0x79, 0x00, 0x00, 0x58, 0x13]

RPM_N0300 = [0x01, 0x06, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00] # CRC CHECK
RPM_N0500 = [0x01, 0x06, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00] # CRC CHECK
RPM_N0700 = [0x01, 0x06, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00] # CRC CHECK
RPM_N1000 = [0x01, 0x06, 0x00, 0x79, 0xFC, 0x18, 0x19, 0x19]
RPM_N2000 = [0x01, 0x06, 0x00, 0x79, 0xF8, 0x30, 0x1B, 0xC7]
RPM_N3000 = [0x01, 0x06, 0x00, 0x79, 0xF4, 0x48, 0x1E, 0xE5]

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
        self.input_rpm_sub = self.create_subscription(
            Int16,
            'input_rpm',
            self.input_rpm_callback,
            10
        )

        # publisher
        self.req_rpm_publisher = self.create_publisher(Int32, 'req_rpm', qos_profile)
        self.cur_rpm_publisher = self.create_publisher(Int32, 'cur_rpm', qos_profile)

        # info timer
        info_timer_period = 1.0 # seconds
        self.info_timer = self.create_timer(info_timer_period, self.info_timer_callback)

        self.get_logger().info("init driving node ! ")


    # info timer
    def info_timer_callback(self):
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
                self.instrument.serial.write(bytes(RPM_P0300))
            elif msg.linear.x < 0.0:
                self.instrument.serial.write(bytes(RPM_N0300))
            else:
                self.instrument.serial.write(bytes(BREAK_ON))
                time.sleep(3)
                self.instrument.serial.write(bytes(BREAK_OFF))
        except Exception as e:
            self.get_logger().error("Fail to write : {}".format(e))

    def input_rpm_callback(self, msg):
        try:
            if msg.data == 300:
                self.instrument.serial.write(bytes(RPM_P0300))
            elif msg.data == 500:
                self.instrument.serial.write(bytes(RPM_P0500))
            elif msg.data == 700:
                self.instrument.serial.write(bytes(RPM_P0700))
            elif msg.data == 1000:
                self.instrument.serial.write(bytes(RPM_P1000))
            elif msg.data == 0:
                self.instrument.serial.write(bytes(BREAK_ON))
                time.sleep(3)
                self.instrument.serial.write(bytes(BREAK_OFF))
            elif msg.data == -1000:
                self.instrument.serial.write(bytes(RPM_N1000))
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

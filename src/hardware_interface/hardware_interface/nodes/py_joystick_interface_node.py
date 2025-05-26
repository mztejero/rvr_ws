#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_types.msg import Joystick


import numpy as np
import serial
import time
import re

class JoystickNode(Node):

    def __init__(self):

        self.dt = 0.1

        super().__init__("py_joystick_interface_node")
        self.counter_ = 0
        self.serial_data = self.create_publisher(Joystick, "/serial", 100)
        self.timer_ = self.create_timer(self.dt, self.send_data)

        self.arduino = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)
        time.sleep(2)
        self.arduino.flush()

        self.joystick_max = 113

    def read_serial(self):
        try:
            read_msg = "A\n"
            self.arduino.write(read_msg.encode('utf-8'))
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode('utf-8').strip()
                # self.get_logger().info(f"{line}")
                if line and line[0] == "V":
                    line = line.strip()
                    data = line.split(':')

                    if len(data) == 6:
                        vr_x = np.float(data[1])
                        vr_y = np.float(data[3])
                        sw = np.float(data[5])

                        # self.get_logger().info(f"VRx = {vr_x}, VRy = {vr_y}, SW = {sw}")
                        return vr_x, vr_y, sw

        except Exception as e:
            print(f"Error in reading: {e}")

    def send_data(self):
        msg = Joystick()
        outputs = self.read_serial()
        if outputs is not None:
            msg.vx = outputs[0]
            msg.vy = outputs[1]
            msg.sw = outputs[2]
            self.serial_data.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = JoystickNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
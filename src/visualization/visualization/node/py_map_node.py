#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_types.msg import Joystick


import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import re

class MapNode(Node):

    def __init__(self):
        super().__init__("py_map_node")

        self.latest_joystick_msg = None

        self.dt = 0.01
        self.k = 0

        self.x_initial = 0
        self.y_initial = 0

        self.x = [self.x_initial]
        self.y = [self.y_initial]

        self.counter_ = 0
        self.joystick_sub = self.create_subscription(Joystick, "/serial", self.joystick_callback, 10)
        self.timer_ = self.create_timer(self.dt, self.draw)

    def joystick_callback(self, msg):
        self.latest_joystick_msg = msg

    def draw(self):
        if self.latest_joystick_msg is not None:
            vx = self.latest_joystick_msg.vx
            vy = self.latest_joystick_msg.vy
            sw = self.latest_joystick_msg.sw

            self.get_logger().info(f"VRx = {vx}, VRy = {vy}, SW = {sw}")

            self.x.append(vx*self.dt + self.x[self.k])
            self.y.append(vy*self.dt + self.y[self.k])

            self.k += 1

            plt.plot(self.x, self.y, color='black')
            plt.pause(0.01)

def main(args = None):
    rclpy.init(args = args)
    node = MapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
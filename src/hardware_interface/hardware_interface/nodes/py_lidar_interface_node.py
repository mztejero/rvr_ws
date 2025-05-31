#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_types.msg import LiDAR

import numpy as np

from hardware_interface.lib.hardware_lib import LiDARInterface

class LiDARInterfaceNode(Node):

    def __init__(self):
        super().__init__("py_lidar_interface_node")

        self.dt = 0.1

        self.lidar = LiDARInterface()
        self.lidar_data = self.create_publisher(LiDAR, "/lidar", 100)
        self.timer = self.create_timer(self.dt, self.LiDAR_publisher)

    def LiDAR_publisher(self):
        msg = LiDAR()

        lidardata = self.lidar.read_raw_data()
        if lidardata is None:
            self.get_logger().info(f"NO DATAAAAAA")

        else:  
            speed, start_angle, distances, intensities, end_angle, time = lidardata

            msg.speed = speed
            msg.start_angle = start_angle
            msg.distances = distances
            msg.intensities = intensities
            msg.end_angle = end_angle
            msg.time = time

            self.lidar_data.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = LiDARInterfaceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
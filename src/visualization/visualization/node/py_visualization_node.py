#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_types.msg import LiDAR

import numpy as np
import matplotlib.pyplot as plt

class VisualizationNode(Node):

    def __init__(self):
        super().__init__("py_visualization_node")

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([-3, 3])
        self.ax.set_ylim([-3, 3])
        self.ax.grid(True)
        
        self.dt = 0.05
        self.x = []
        self.y = []
        self.lap_complete = False
        self.prev_end_angle = None
        self.lidar_subscriber = self.create_subscription(LiDAR, "/lidar", self.lidar_callback, 10)

    def lidar_callback(self, msg = LiDAR):
        # self.get_logger().info(f'angles: {msg.start_angle}, {msg.end_angle}')

        start_angle = msg.start_angle
        end_angle = msg.end_angle
        distances = msg.distances
        angles = np.linspace(start_angle, end_angle, len(distances))

        self.x.extend(distances*np.cos(angles))
        self.y.extend(distances*np.sin(angles))

        if end_angle is not None and self.prev_end_angle is not None:
            delta_angle = abs(end_angle - self.prev_end_angle)
            self.get_logger().info(f"delta end angle = {delta_angle}")
            if end_angle < 2.0 and  self.prev_end_angle > 5.0 and delta_angle > 2.0:
                self.lap_complete = True
                self.get_logger().info(f"LAP COMPLETE")
        self.get_logger().info(f"previous end angle = {self.prev_end_angle}, end angle = {end_angle}")
        self.prev_end_angle = end_angle

        if self.lap_complete:
            self.x = self.x[len(distances):]
            self.y = self.y[len(distances):]

            self.ax.clear()
            self.ax.set_xlim([-3, 3])
            self.ax.set_ylim([-3, 3])
            self.ax.grid(True)

        self.ax.scatter(self.x, self.y, s=3, color="blue")
        plt.pause(self.dt)
        self.lap_complete = False

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
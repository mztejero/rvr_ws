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
        
        self.dt = 0.01
        self.x = []
        self.y = []
        self.prev_end_angle = None
        self.lidar_subscriber = self.create_subscription(LiDAR, "/lidar", self.lidar_callback, 10)

    def lidar_callback(self, msg = LiDAR):
        start_angle = msg.start_angle
        end_angle = msg.end_angle
        distances = msg.distances
        angles = np.linspace(start_angle, end_angle, len(distances))

        self.x.extend(distances*np.cos(angles))
        self.y.extend(distances*np.sin(angles))

        distance_std = np.std(distances)
        if distance_std < 0.01:
            return

        if end_angle is not None and self.prev_end_angle is not None:
            delta_angle = abs(end_angle - self.prev_end_angle) # account for noisy data
            if end_angle < 2.0 and  self.prev_end_angle > 5.0 and delta_angle > 2.0: # check if new lap is complete
                self.ax.clear()
                self.ax.set_xlim([-3, 3])
                self.ax.set_ylim([-3, 3])
                self.ax.grid(True)
                self.ax.scatter(self.x, self.y, s=3, color="blue")
                plt.pause(self.dt)
                self.x = []
                self.y = []
        self.prev_end_angle = end_angle

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_types.msg import IMUdata, Vector3

from rvr_ws.src.hardware_interface.hardware_interface.lib.hardware_lib import BLEInterface


class BLEInterfaceNode(Node):

    def __init__(self):
        super().__init__("py_hardware_interface_node")

        self.dt = 0.1

        self.ble = BLEInterface()

        self.serial_data = self.create_publisher(IMUdata, "/imu", 100)
        self.timer = self.create_timer(self.dt, self.IMU_publisher)

    def IMU_publisher(self):
        msg = IMUdata()

        outputs = self.ble.read_serial()

        if outputs is not None:
            mag, a, w = outputs

            msg.mag = Vector3(x = float(mag[0]), y = float(mag[1]), z = float(mag[2]))
            msg.a = Vector3(x = float(a[0]), y = float(a[1]), z = float(a[2]))
            msg.w = Vector3(x = float(w[0]), y = float(w[1]), z = float(w[2]))

            self.serial_data.publish(msg)
        
def main(args = None):
    rclpy.init(args = args)
    node = BLEInterfaceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
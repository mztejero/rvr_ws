from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nodes = [
        # Node(
        #     package = "hardware_interface",
        #     executable = "py_lidar_interface_node",
        #     name = "lidar_node",
        #     output = "screen"
        # ),
        Node(
            package = "visualization",
            executable = "py_visualization_node",
            name = "visualization_node",
            output = "screen"
        )
    ]

    ld = LaunchDescription(nodes)

    return ld
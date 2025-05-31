from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nodes = [
        Node(
            package = "hardware_interface",
            executable = "py_joystick_interface_node",
            name = "remote_control_node",
            output = "screen"
        ),
        Node(
            package = "visualization",
            executable = "py_map_node",
            name = "remote_control_node",
            output = "screen"
        )
    ]

    ld = LaunchDescription(nodes)

    return ld
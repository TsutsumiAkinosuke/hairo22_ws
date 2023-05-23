from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy_linux",
            namespace="joy_linux",
            executable="joy_linux_node"
        ),
        Node(
            package="cmd_pub_node",
            namespace="cmd_pub_node",
            executable="convert_joy2cmd"
        )
    ])
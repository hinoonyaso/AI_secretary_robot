from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory("battery_cpp"), "config", "params.yaml")

    return LaunchDescription([
        Node(
            package="battery_cpp",
            executable="battery_node",
            name="battery_node",
            output="screen",
            parameters=[params],
        )
    ])

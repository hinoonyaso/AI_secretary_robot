import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("stt_cpp")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    return LaunchDescription([
        Node(
            package="stt_cpp",
            executable="stt_node",
            name="stt_node",
            output="screen",
            parameters=[params_file],
        ),
    ])

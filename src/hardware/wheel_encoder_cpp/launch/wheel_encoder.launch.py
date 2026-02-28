import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("wheel_encoder_cpp"),
        "config",
        "params.yaml",
    )

    return LaunchDescription([
        Node(
            package="wheel_encoder_cpp",
            executable="wheel_encoder_node",
            name="wheel_encoder_node",
            output="screen",
            parameters=[params_file],
        )
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("intent_router_cpp")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    return LaunchDescription([
        Node(
            package="intent_router_cpp",
            executable="intent_router_node",
            name="intent_router_node",
            output="screen",
            parameters=[params_file],
        ),
    ])

#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_config_dir = get_package_share_directory("slam_config")
    config_file = os.path.join(slam_config_dir, "config", "slam_params.yaml")
    default_map_file = os.path.join(slam_config_dir, "maps", "saved_map.posegraph")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    declare_map_file = DeclareLaunchArgument(
        "map_file",
        default_value=default_map_file,
        description="Full path to serialized map (.posegraph) file",
    )

    localization_node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
            {"mode": "localization"},
            {"map_file_name": map_file},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_file)
    ld.add_action(localization_node)
    return ld

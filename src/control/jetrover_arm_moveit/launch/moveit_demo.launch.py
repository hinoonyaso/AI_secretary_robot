#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    full_path = os.path.join(package_path, relative_path)
    with open(full_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    pkg_name = "jetrover_arm_moveit"
    pkg_share = get_package_share_directory(pkg_name)

    default_xacro = os.path.join(pkg_share, "urdf", "jetrover.xacro")

    xacro_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value=default_xacro,
        description="Absolute path to robot xacro (default uses jetrover_arm_moveit/urdf/jetrover.xacro)",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_share, "config", "moveit_default.rviz"),
        description="Absolute path to RViz config",
    )
    fake_joint_state_arg = DeclareLaunchArgument(
        "use_fake_joint_states",
        default_value="true",
        description="Use joint_state_publisher for RViz/demo; set false for real hardware servo bridge",
    )
    servo_calibration_arg = DeclareLaunchArgument(
        "servo_calibration_yaml",
        default_value=os.path.join(pkg_share, "config", "servo_calibration.yaml"),
        description="YAML file with arm_servo_state_bridge calibration params",
    )
    intent_bridge_arg = DeclareLaunchArgument(
        "enable_intent_arm_bridge",
        default_value="true",
        description="Enable intent->arm trajectory bridge node",
    )

    robot_description = {
        "robot_description": Command(
            [FindExecutable(name="xacro"), " ", LaunchConfiguration("xacro_path")]
        )
    }

    srdf_path = os.path.join(pkg_share, "config", "jetrover_arm.srdf")
    with open(srdf_path, "r", encoding="utf-8") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(pkg_name, "config/kinematics.yaml")
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(pkg_name, "config/joint_limits.yaml")["joint_limits"]
    }

    ompl_yaml = load_yaml(pkg_name, "config/ompl_planning.yaml")
    # Normalize ompl YAML to avoid accidental ompl->ompl double nesting.
    # Expected source layout:
    #   ompl: {planning_plugin, request_adapters, ...}
    #   arm:  {planner_configs: [...]}
    ompl_pipeline_cfg = dict(ompl_yaml.get("ompl", {}))
    for group_name, group_cfg in ompl_yaml.items():
        if group_name == "ompl":
            continue
        ompl_pipeline_cfg[group_name] = group_cfg
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_pipeline_cfg,
    }

    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_controllers = load_yaml(pkg_name, "config/moveit_controllers.yaml")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[("joint_states", "/moveit_joint_states")],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            trajectory_execution,
            planning_scene_monitor_parameters,
            moveit_controllers,
            {"use_sim_time": False},
        ],
    )

    # Publish /tf and /joint_states so RViz can render the robot model.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[("joint_states", "/moveit_joint_states")],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        arguments=[LaunchConfiguration("xacro_path")],
        remappings=[("joint_states", "/moveit_joint_states")],
        condition=IfCondition(LaunchConfiguration("use_fake_joint_states")),
    )

    arm_servo_state_bridge_node = Node(
        package=pkg_name,
        executable="arm_servo_state_bridge.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "service_name": "/ros_robot_controller/bus_servo/get_state",
                "publish_topic": "/moveit_joint_states",
                "publish_hz": 10.0,
                "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"],
                "servo_ids": [1, 2, 3, 4, 5, 10],
            },
            LaunchConfiguration("servo_calibration_yaml"),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_fake_joint_states")),
    )

    follow_joint_trajectory_bridge_node = Node(
        package=pkg_name,
        executable="follow_joint_trajectory_bridge.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "publish_topic": "/ros_robot_controller/bus_servo/set_position",
                "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"],
                "servo_ids": [1, 2, 3, 4, 5, 10],
            },
            LaunchConfiguration("servo_calibration_yaml"),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_fake_joint_states")),
    )

    intent_arm_bridge_node = Node(
        package=pkg_name,
        executable="intent_arm_bridge_node",
        output="screen",
        parameters=[
            {
                "intent_category_topic": "/intent_router/category",
                "intent_command_topic": "/intent_router/robot_command",
                "trajectory_action_name": "/arm_controller/follow_joint_trajectory",
                "trajectory_duration_sec": 1.5,
                "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"],
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_intent_arm_bridge")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
        ],
    )

    return LaunchDescription([
        xacro_arg,
        rviz_config_arg,
        fake_joint_state_arg,
        servo_calibration_arg,
        intent_bridge_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        arm_servo_state_bridge_node,
        follow_joint_trajectory_bridge_node,
        intent_arm_bridge_node,
        move_group_node,
        rviz_node,
    ])

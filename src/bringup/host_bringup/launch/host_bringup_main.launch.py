#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam = LaunchConfiguration("use_slam")
    use_localization = LaunchConfiguration("use_localization")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_arm = LaunchConfiguration("use_arm")
    use_camera = LaunchConfiguration("use_camera")
    use_monitor = LaunchConfiguration("use_monitor")
    autostart_nav2 = LaunchConfiguration("autostart_nav2")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_frame = LaunchConfiguration("lidar_frame")
    motor_device = LaunchConfiguration("motor_device")
    motor_baudrate = LaunchConfiguration("motor_baudrate")
    map_file = LaunchConfiguration("map_file")
    nav2_map_file = LaunchConfiguration("nav2_map_file")
    xacro_path = LaunchConfiguration("xacro_path")

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_path])
    }

    # Phase 1: hardware drivers (immediate)
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_robot_controller_cpp"), "launch", "ros_robot_controller_cpp.launch.py"]
            )
        ),
        launch_arguments={
            "device": motor_device,
            "baudrate": motor_baudrate,
        }.items(),
    )

    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rplidar_ros"), "launch", "rplidar_a1_launch.py"])
        ),
        launch_arguments={
            "serial_port": lidar_port,
            "frame_id": lidar_frame,
        }.items(),
    )

    lidar_monitor_node = Node(
        package="lidar_cpp",
        executable="lidar_node",
        name="lidar_monitor",
        output="screen",
        parameters=[{"scan_topic": "/scan", "log_every_n": 20}],
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("imu_bridge_cpp"), "launch", "imu_bridge.launch.py"])
        )
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("battery_cpp"), "launch", "battery.launch.py"])
        )
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("depth_camera_cpp"), "launch", "dabai_dcw_usb2.launch.py"])
        ),
        condition=IfCondition(use_camera),
    )

    # Phase 2: SLAM (delayed)
    slam_mapping_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("slam_config"), "launch", "online_async_launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
        condition=IfCondition(
            PythonExpression(["'", use_slam, "' == 'true' and '", use_localization, "' == 'false'"])
        ),
    )

    slam_localization_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("slam_config"), "launch", "localization_launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map_file": map_file,
                }.items(),
            )
        ],
        condition=IfCondition(
            PythonExpression(["'", use_slam, "' == 'true' and '", use_localization, "' == 'true'"])
        ),
    )

    # Phase 3: Nav2 (delayed)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_config"), "launch", "nav2_bringup.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart_nav2,
                    "map": nav2_map_file,
                }.items(),
            )
        ],
        condition=IfCondition(
            PythonExpression(["'", use_nav2, "' == 'true' and '", use_localization, "' == 'true'"])
        ),
    )

    # Phase 4: MoveIt2 (optional, delayed)
    moveit_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("jetrover_arm_moveit"), "launch", "moveit_demo.launch.py"])
                ),
                launch_arguments={
                    "use_fake_joint_states": "false",
                    "enable_intent_arm_bridge": "true",
                }.items(),
            )
        ],
        condition=IfCondition(use_arm),
    )

    system_monitor = Node(
        package="host_bringup",
        executable="host_system_monitor.py",
        name="host_system_monitor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_monitor),
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_slam", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_localization", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_nav2", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_arm", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_camera", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_monitor", default_value="true"))
    ld.add_action(DeclareLaunchArgument("autostart_nav2", default_value="true"))
    ld.add_action(DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"))
    ld.add_action(DeclareLaunchArgument("lidar_frame", default_value="laser_frame"))
    ld.add_action(DeclareLaunchArgument("motor_device", default_value="/dev/rrc"))
    ld.add_action(DeclareLaunchArgument("motor_baudrate", default_value="1000000"))
    ld.add_action(
        DeclareLaunchArgument(
            "xacro_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("jetrover_arm_moveit"), "urdf", "jetrover.xacro"]
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "map_file",
            default_value=PathJoinSubstitution([FindPackageShare("slam_config"), "maps", "saved_map.posegraph"]),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "nav2_map_file",
            default_value=PathJoinSubstitution([FindPackageShare("nav2_config"), "maps", "default_map.yaml"]),
        )
    )

    ld.add_action(motor_launch)
    ld.add_action(lidar_driver_launch)
    ld.add_action(lidar_monitor_node)
    ld.add_action(imu_launch)
    ld.add_action(battery_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(camera_launch)
    ld.add_action(slam_mapping_launch)
    ld.add_action(slam_localization_launch)
    ld.add_action(nav2_launch)
    ld.add_action(moveit_launch)
    ld.add_action(system_monitor)

    return ld

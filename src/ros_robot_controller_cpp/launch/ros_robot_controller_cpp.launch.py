from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = LaunchConfiguration('device', default='/dev/rrc')
    baudrate = LaunchConfiguration('baudrate', default='1000000')
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    auto_detect_serial = LaunchConfiguration('auto_detect_serial', default='true')

    kill_stale = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -x ros_robot_controller_cpp_node 2>/dev/null || true; sleep 0.3; '
             'pkill -9 -x ros_robot_controller_cpp_node 2>/dev/null || true; sleep 0.2; true'],
        output='screen',
    )

    start_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_robot_controller_cpp',
                executable='ros_robot_controller_cpp_node',
                output='screen',
                parameters=[{
                    'device': device,
                    'baudrate': ParameterValue(baudrate, value_type=int),
                    'imu_frame': imu_frame,
                    'auto_detect_serial': ParameterValue(auto_detect_serial, value_type=bool),
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value=device),
        DeclareLaunchArgument('baudrate', default_value=baudrate),
        DeclareLaunchArgument('imu_frame', default_value=imu_frame),
        DeclareLaunchArgument('auto_detect_serial', default_value=auto_detect_serial),
        kill_stale,
        start_node,
    ])

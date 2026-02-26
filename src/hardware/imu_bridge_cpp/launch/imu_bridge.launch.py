from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_bridge_cpp',
            executable='imu_bridge_node',
            name='imu_bridge_node',
            output='screen',
            parameters=[{
                'input_topic': '/ros_robot_controller/imu_raw',
                'output_topic': '/imu/data',
                'frame_id': 'imu_link',
                'timeout_sec': 1.0,
            }]
        )
    ])

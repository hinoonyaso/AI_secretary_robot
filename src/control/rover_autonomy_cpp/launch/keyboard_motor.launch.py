from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_autonomy_cpp',
            executable='keyboard_motor_node',
            name='keyboard_motor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'left_motor_ids': [1, 2],
                'right_motor_ids': [3, 4],
            }],
        )
    ])

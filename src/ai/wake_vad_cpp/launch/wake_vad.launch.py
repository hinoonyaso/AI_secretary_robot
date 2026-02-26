import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wake_vad_cpp')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    keyword_path = PathJoinSubstitution([pkg_share, 'wake_sound'])
    model_path = PathJoinSubstitution([pkg_share, 'resource', 'porcupine_params_ko.pv'])
    vad_model_path = PathJoinSubstitution([pkg_share, 'resource', 'silero_vad.onnx'])
    wav_output_dir = os.path.join(pkg_share, 'record_sound')

    return LaunchDescription([
        Node(
            package='wake_vad_cpp',
            executable='wake_vad_node',
            name='wake_vad_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'keyword_path': keyword_path,
                    'porcupine_model_path': model_path,
                    'vad_model_path': vad_model_path,
                    'wav_output_dir': wav_output_dir,
                },
            ],
        ),
    ])

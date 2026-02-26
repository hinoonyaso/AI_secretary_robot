from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wake_vad_launch = PathJoinSubstitution(
        [FindPackageShare("wake_vad_cpp"), "launch", "wake_vad.launch.py"])
    stt_launch = PathJoinSubstitution(
        [FindPackageShare("stt_cpp"), "launch", "stt.launch.py"])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(wake_vad_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(stt_launch)),
    ])

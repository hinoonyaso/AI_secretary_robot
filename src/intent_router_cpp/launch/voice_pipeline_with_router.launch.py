from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wake_stt_launch = PathJoinSubstitution(
        [FindPackageShare("stt_cpp"), "launch", "wake_vad_with_stt.launch.py"])
    router_launch = PathJoinSubstitution(
        [FindPackageShare("intent_router_cpp"), "launch", "intent_router.launch.py"])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(wake_stt_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(router_launch)),
    ])

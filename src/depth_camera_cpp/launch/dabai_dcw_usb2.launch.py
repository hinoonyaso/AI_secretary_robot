from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    camera_name = LaunchConfiguration("camera_name", default="depth_cam").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix", default="").perform(context)

    args = [
        DeclareLaunchArgument("camera_name", default_value=camera_name),
        DeclareLaunchArgument("tf_prefix", default_value=tf_prefix),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("usb_port", default_value=""),
        DeclareLaunchArgument("device_num", default_value="1"),
        DeclareLaunchArgument("vendor_id", default_value="0x2bc5"),
        DeclareLaunchArgument("product_id", default_value="0x0659"),
        DeclareLaunchArgument("connection_delay", default_value="100"),
        DeclareLaunchArgument("retry_on_usb3_detection_failure", default_value="false"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_ir", default_value="false"),
        DeclareLaunchArgument("enable_ir_auto_exposure", default_value="false"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="360"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument("color_format", default_value="MJPG"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="360"),
        DeclareLaunchArgument("depth_fps", default_value="30"),
        DeclareLaunchArgument("depth_format", default_value="Y11"),
        DeclareLaunchArgument("depth_registration", default_value="true"),
        DeclareLaunchArgument("align_mode", default_value="HW"),
        DeclareLaunchArgument("enable_point_cloud", default_value="true"),
        DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("tf_publish_rate", default_value="10.0"),
        DeclareLaunchArgument("log_level", default_value="none"),
    ]

    param_names = [
        "depth_registration",
        "serial_number",
        "usb_port",
        "device_num",
        "vendor_id",
        "product_id",
        "enable_point_cloud",
        "enable_colored_point_cloud",
        "connection_delay",
        "retry_on_usb3_detection_failure",
        "color_width",
        "color_height",
        "color_fps",
        "color_format",
        "enable_color",
        "depth_width",
        "depth_height",
        "depth_fps",
        "depth_format",
        "enable_depth",
        "enable_ir",
        "enable_ir_auto_exposure",
        "publish_tf",
        "tf_publish_rate",
        "log_level",
        "align_mode",
    ]

    parameters = []
    for name in param_names:
        if name == "product_id":
            parameters.append({name: ParameterValue(LaunchConfiguration(name), value_type=str)})
        else:
            parameters.append({name: LaunchConfiguration(name)})

    driver = ComposableNode(
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        name=camera_name,
        namespace="",
        parameters=parameters,
        remappings=[
            ("/tf", "/" + tf_prefix + "tf"),
            ("/tf_static", "/" + tf_prefix + "tf_static"),
            ("/" + camera_name + "/color/camera_info", "/" + camera_name + "/rgb/camera_info"),
            ("/" + camera_name + "/color/image_raw", "/" + camera_name + "/rgb/image_raw"),
            ("/" + camera_name + "/color/image_raw/compressed", "/" + camera_name + "/rgb/image_raw/compressed"),
            ("/" + camera_name + "/color/image_raw/compressedDepth", "/" + camera_name + "/rgb/image_raw/compressedDepth"),
            ("/" + camera_name + "/depth/color/points", "/" + camera_name + "/depth_registered/points"),
        ],
    )

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[driver],
        output="screen",
    )

    monitor = Node(
        package="depth_camera_cpp",
        executable="depth_camera_monitor",
        name="depth_camera_monitor",
        output="screen",
        parameters=[{"camera_name": LaunchConfiguration("camera_name")}],
    )

    grouped = GroupAction(actions=[PushRosNamespace(camera_name), container, monitor])

    return args + [grouped]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

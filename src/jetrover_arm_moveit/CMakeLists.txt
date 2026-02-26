cmake_minimum_required(VERSION 3.10)
project(jetrover_arm_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

add_executable(intent_arm_bridge_node src/intent_arm_bridge_node.cpp)
ament_target_dependencies(
  intent_arm_bridge_node
  rclcpp
  rclcpp_action
  std_msgs
  control_msgs
  trajectory_msgs
  ros_robot_controller_msgs
)

install(TARGETS
  intent_arm_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf meshes DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/arm_moveit_commander.py
  scripts/arm_servo_state_bridge.py
  scripts/capture_servo_centers.py
  scripts/follow_joint_trajectory_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

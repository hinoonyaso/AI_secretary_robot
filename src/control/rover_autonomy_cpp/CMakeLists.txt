cmake_minimum_required(VERSION 3.10)
project(rover_autonomy_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

add_executable(keyboard_motor_node
  src/keyboard_motor_node.cpp
)

ament_target_dependencies(
  keyboard_motor_node
  geometry_msgs
  rclcpp
  ros_robot_controller_msgs
)

target_compile_features(keyboard_motor_node PUBLIC cxx_std_17)

install(TARGETS
  keyboard_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/keyboard_motor.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

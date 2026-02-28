cmake_minimum_required(VERSION 3.10)
project(wheel_encoder_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wheel_encoder_node
  src/wheel_encoder_node.cpp
)

ament_target_dependencies(
  wheel_encoder_node
  rclcpp
  ros_robot_controller_msgs
  sensor_msgs
  std_msgs
)

target_compile_features(wheel_encoder_node PUBLIC cxx_std_17)

install(TARGETS
  wheel_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/wheel_encoder.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  config/params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

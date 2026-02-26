cmake_minimum_required(VERSION 3.10)
project(ros_robot_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

add_executable(ros_robot_controller_cpp_node
  src/ros_robot_controller_cpp_node.cpp
  src/board.cpp
)
ament_target_dependencies(
  ros_robot_controller_cpp_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  ros_robot_controller_msgs
)

target_compile_features(ros_robot_controller_cpp_node PUBLIC cxx_std_17)

install(TARGETS
  ros_robot_controller_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/ros_robot_controller_cpp.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

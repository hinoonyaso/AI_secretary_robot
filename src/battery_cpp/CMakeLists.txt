cmake_minimum_required(VERSION 3.10)
project(battery_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(battery_node
  src/battery_node.cpp
)

target_compile_features(battery_node PUBLIC cxx_std_17)
ament_target_dependencies(
  battery_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS battery_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

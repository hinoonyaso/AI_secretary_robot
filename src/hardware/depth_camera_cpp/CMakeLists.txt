cmake_minimum_required(VERSION 3.10)
project(depth_camera_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(depth_camera_monitor src/depth_camera_monitor.cpp)
target_compile_features(depth_camera_monitor PUBLIC cxx_std_17)
ament_target_dependencies(depth_camera_monitor rclcpp sensor_msgs)

install(TARGETS
  depth_camera_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


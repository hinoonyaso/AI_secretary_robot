cmake_minimum_required(VERSION 3.10)
project(jetrover_web_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

add_executable(udp_servo_bridge_node src/udp_servo_bridge_node.cpp)
ament_target_dependencies(udp_servo_bridge_node rclcpp ros_robot_controller_msgs)

target_compile_features(udp_servo_bridge_node PUBLIC cxx_std_17)

install(TARGETS
  udp_servo_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.10)
project(slam_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/save_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

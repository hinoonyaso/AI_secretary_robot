cmake_minimum_required(VERSION 3.10)
project(nav2_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

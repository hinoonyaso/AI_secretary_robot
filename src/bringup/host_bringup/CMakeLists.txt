cmake_minimum_required(VERSION 3.10)
project(host_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/start_host.sh
    scripts/host_system_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

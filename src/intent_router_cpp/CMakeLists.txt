cmake_minimum_required(VERSION 3.10)
project(intent_router_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CURL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_common REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(intent_router_node
  src/intent_router_node.cpp
)

target_include_directories(intent_router_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(intent_router_node PUBLIC cxx_std_17)
ament_target_dependencies(intent_router_node ament_index_cpp rclcpp rover_common std_msgs)
if(TARGET CURL::libcurl)
  target_link_libraries(intent_router_node CURL::libcurl)
else()
  target_link_libraries(intent_router_node ${CURL_LIBRARIES})
endif()

install(TARGETS
  intent_router_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.10)
project(llm_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

add_executable(llm_node
  src/llm_node.cpp
  src/llm_engine.cpp
)

target_include_directories(llm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(llm_node PUBLIC cxx_std_17)
ament_target_dependencies(llm_node rclcpp rover_common std_msgs)

if(TARGET CURL::libcurl)
  target_link_libraries(llm_node CURL::libcurl)
else()
  target_link_libraries(llm_node ${CURL_LIBRARIES})
endif()

install(TARGETS
  llm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

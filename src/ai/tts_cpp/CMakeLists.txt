cmake_minimum_required(VERSION 3.10)
project(tts_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_common REQUIRED)
find_package(std_msgs REQUIRED)
pkg_check_modules(ALSA alsa)
pkg_check_modules(SHERPA_ONNX REQUIRED sherpa-onnx)

add_executable(tts_node
  src/tts_node.cpp
  src/tts_engine.cpp
  src/alsa_player.cpp
  src/piper_onnx.cpp
)

target_include_directories(tts_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(tts_node PUBLIC cxx_std_17)
ament_target_dependencies(tts_node ament_index_cpp rclcpp rover_common std_msgs)

if(TARGET CURL::libcurl)
  target_link_libraries(tts_node CURL::libcurl)
else()
  target_link_libraries(tts_node ${CURL_LIBRARIES})
endif()

if(ALSA_FOUND)
  target_compile_definitions(tts_node PRIVATE HAVE_ALSA=1)
  target_include_directories(tts_node PRIVATE ${ALSA_INCLUDE_DIRS})
  target_link_libraries(tts_node ${ALSA_LIBRARIES})
endif()

target_include_directories(tts_node PRIVATE ${SHERPA_ONNX_INCLUDE_DIRS})
target_link_directories(tts_node PRIVATE ${SHERPA_ONNX_LIBRARY_DIRS})
target_link_libraries(tts_node ${SHERPA_ONNX_LIBRARIES})
set_target_properties(tts_node PROPERTIES
  BUILD_RPATH "${SHERPA_ONNX_LIBRARY_DIRS}"
  INSTALL_RPATH "${SHERPA_ONNX_LIBRARY_DIRS}"
)

install(TARGETS
  tts_node
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

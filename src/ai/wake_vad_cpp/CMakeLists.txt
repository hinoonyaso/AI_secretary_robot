cmake_minimum_required(VERSION 3.10)
project(wake_vad_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

set(PORCUPINE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SDK/porcupine")
set(ONNXRUNTIME_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SDK/onnxruntime-linux-aarch64-1.24.1")

set(PORCUPINE_INCLUDE_DIR "${PORCUPINE_SDK_DIR}/include")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_SDK_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_SDK_DIR}/lib")
set(ONNXRUNTIME_LIB "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so")

set(PORCUPINE_LIB_CANDIDATES
  "${PORCUPINE_SDK_DIR}/lib/raspberry-pi/cortex-a53-aarch64/libpv_porcupine.so"
  "${PORCUPINE_SDK_DIR}/lib/raspberry-pi/cortex-a72-aarch64/libpv_porcupine.so"
  "${PORCUPINE_SDK_DIR}/lib/raspberry-pi/cortex-a76-aarch64/libpv_porcupine.so"
)
set(PORCUPINE_LIB "")
foreach(candidate ${PORCUPINE_LIB_CANDIDATES})
  if(EXISTS "${candidate}")
    set(PORCUPINE_LIB "${candidate}")
    break()
  endif()
endforeach()

if(NOT EXISTS "${PORCUPINE_INCLUDE_DIR}/pv_porcupine.h")
  message(FATAL_ERROR "Porcupine header not found: ${PORCUPINE_INCLUDE_DIR}/pv_porcupine.h")
endif()
if(NOT PORCUPINE_LIB)
  message(FATAL_ERROR "Porcupine shared library not found under ${PORCUPINE_SDK_DIR}/lib")
endif()
if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime header not found: ${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h")
endif()
if(NOT EXISTS "${ONNXRUNTIME_LIB}")
  message(FATAL_ERROR "ONNX Runtime shared library not found: ${ONNXRUNTIME_LIB}")
endif()

add_executable(wake_vad_node
  src/wake_vad_node.cpp
  src/audio_input.cpp
  src/porcupine_engine.cpp
  src/vad_engine.cpp
  src/state_machine.cpp
  src/wav_writer.cpp
)

target_include_directories(wake_vad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PORTAUDIO_INCLUDE_DIRS}
  ${PORCUPINE_INCLUDE_DIR}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

target_compile_features(wake_vad_node PUBLIC cxx_std_17)
ament_target_dependencies(wake_vad_node rclcpp ros_robot_controller_msgs std_msgs)
target_link_libraries(wake_vad_node
  ${PORTAUDIO_LIBRARIES}
  ${PORCUPINE_LIB}
  ${ONNXRUNTIME_LIB}
)
set_target_properties(wake_vad_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIB_DIR};${CMAKE_CURRENT_BINARY_DIR}"
  INSTALL_RPATH "\$ORIGIN"
)

install(TARGETS
  wake_vad_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${PORCUPINE_LIB}
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so"
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1"
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1.24.1"
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime_providers_shared.so"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config resource wake_sound
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

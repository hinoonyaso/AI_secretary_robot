cmake_minimum_required(VERSION 3.10)
project(ros_robot_controller_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
# 아래 구간의 주석을 해제하면
# 추가 의존성을 수동으로 지정할 수 있습니다.
# find_package(<의존성> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AudioBuffer.msg"
    "msg/BusServoState.msg"
    "msg/ButtonState.msg"
    "msg/BuzzerState.msg"
    "msg/LedState.msg"
    "msg/MotorState.msg"
    "msg/MotorsState.msg"
    "msg/PWMServoState.msg"
    "msg/Sbus.msg"
    "msg/OLEDState.msg"
    "msg/SetBusServoState.msg"
    "msg/ServoPosition.msg"
    "msg/ServosPosition.msg"
    "msg/SetPWMServoState.msg"
    "msg/GetBusServoCmd.msg"
    "msg/GetPWMServoCmd.msg"
    "srv/GetBusServoState.srv"
    "srv/GetPWMServoState.srv"
    DEPENDENCIES std_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 아래 줄은 저작권 검사 린터를 건너뜁니다
  # 모든 소스 파일에 저작권/라이선스가 없을 때 주석 해제하세요
  #set(ament_cmake_copyright_FOUND TRUE)
  # 아래 줄은 cpplint를 건너뜁니다(깃 저장소에서만 동작)
  # 이 패키지가 깃 저장소가 아닐 때 주석 해제하세요
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

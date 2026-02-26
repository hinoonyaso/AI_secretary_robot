# IMU Driver 원리와 구현 가이드

## 1. 왜 "드라이버"가 필요한가
- `imu_bridge_cpp`는 IMU 메시지를 **중계/가공**하는 노드다.
- 실제 하드웨어 데이터(센서값)를 만들어내는 건 **드라이버(Producer)** 다.
- 드라이버가 없으면 입력 토픽이 비어 있고, 브리지는 정상이어도 출력할 데이터가 없다.

현재 구조 예시:
- `STM32(IMU)` -> `serial(/dev/rrc)` -> `ros_robot_controller` -> `/ros_robot_controller/imu_raw` -> `imu_bridge_cpp` -> `/imu/data`

## 2. 드라이버의 핵심 역할
- 물리 통신 연결: UART/USB/Can 등 디바이스 오픈, baudrate/timeout 설정
- 프레임 수신: 바이트 스트림에서 패킷 경계 찾기
- 무결성 검사: CRC/checksum 검증
- 메시지 파싱: 바이트 -> 물리값(rad/s, m/s^2) 변환
- ROS 메시지 발행: `sensor_msgs/msg/Imu`
- 장애 대응: 재연결, 타임아웃 경고, 예외 복구

## 3. 동작 원리(런타임 루프)
1. 포트 오픈 (`/dev/rrc`, `1000000` 등)
2. 수신 버퍼 누적
3. 패킷 헤더/길이 기준으로 프레임 추출
4. CRC 검사 통과 시 IMU payload 파싱
5. 단위/축 보정 후 `sensor_msgs/Imu` 생성
6. 타임스탬프/프레임 ID 채워 publish
7. 주기적으로 상태 로그(rate, timeout) 출력
8. 오류 시 포트 재초기화 후 재시도

## 4. 구현 방법 선택지

## 방법 A: 기존 드라이버 재사용 (권장)
- `ros_robot_controller`를 워크스페이스에 포함하고 그대로 실행
- 장점: 빠름, 안정적, 검증된 경로
- 단점: 외부 패키지 의존

적합한 경우:
- 빠르게 시스템을 살려야 할 때
- 프로토콜 상세를 모를 때

## 방법 B: 새 드라이버 직접 구현
- C++ 또는 Python으로 시리얼 통신 + 파서 + ROS publish 구현
- 장점: 구조 단순화, 제어권 높음
- 단점: 공수 큼, 디버깅 부담 큼

적합한 경우:
- 프로토콜 문서/펌웨어 지원이 충분할 때
- 장기적으로 외부 의존을 줄여야 할 때

## 5. 직접 구현 시 권장 아키텍처

구성:
- `SerialReader`: 포트 관리(read/reconnect)
- `FrameParser`: state machine(헤더/길이/CRC)
- `ImuConverter`: raw -> SI 단위 변환, 축 정렬
- `ImuPublisher`: ROS2 `sensor_msgs/Imu` 발행
- `HealthMonitor`: rate/timeout 진단

스레딩:
- 수신 스레드 1개 + ROS executor
- 공유 데이터는 lock-free queue 또는 mutex로 보호

QoS:
- 센서 토픽은 기본적으로 `SensorDataQoS` 권장

## 6. 최소 구현 체크리스트
- [ ] 토픽명: 입력/출력 명확화 (`/ros_robot_controller/imu_raw`, `/imu/data`)
- [ ] frame_id 일관성 (`imu_link`)
- [ ] timestamp 0 방지
- [ ] covariance 기본값 설정
- [ ] 포트 끊김 자동 재연결
- [ ] 데이터 미수신 경고(예: 1초 timeout)
- [ ] `ros2 topic hz/echo`로 수신 검증

## 7. 디버깅 순서(실전)
1. 디바이스 확인: `ls /dev/rrc*`
2. 포트 권한/점유 확인: `groups`, `lsof /dev/rrc`
3. 원본 토픽 확인: `ros2 topic echo /ros_robot_controller/imu_raw --once`
4. 주기 확인: `ros2 topic hz /ros_robot_controller/imu_raw`
5. 브리지 확인: `ros2 topic echo /imu/data --once`
6. 로그 확인: timeout/no telemetry 문구 확인

## 8. 현재 프로젝트에 적용한 결론
- `rover_ws`에서 IMU를 받으려면, 브리지만으로는 부족하다.
- 반드시 드라이버 노드(예: `ros_robot_controller`)가 먼저 떠서 raw IMU를 publish해야 한다.
- 그 위에 `imu_bridge_cpp`를 연결하면 `/imu/data`로 정상 전달된다.

## 9. 다음 확장 아이디어
- `imu_bridge_cpp`에 `direct_serial_mode` 추가
- 모드 1: `bridge_mode` (토픽 입력 받아 중계)
- 모드 2: `driver_mode` (시리얼 직접 읽어 publish)
- 런타임 파라미터로 모드 선택 가능하게 설계

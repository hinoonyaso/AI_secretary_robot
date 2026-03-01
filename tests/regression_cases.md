# Regression Cases

## Case 1: Serial 패킷 드랍
- 문제: serial 통신 중 패킷 유실/응답 지연
- 조건: 관련 launch 3회 연속 재시작
- 권장 명령:
  - `ros2 launch ros_robot_controller_cpp ros_robot_controller_cpp.launch.py`
- 기대 결과:
  - 노드 정상 기동
  - 치명적 에러 없이 통신 유지

## Case 2: MoveIt 계획 실패
- 문제: MoveIt planning 실패 또는 trajectory 브리지 이상
- 조건: MoveIt 데모 기동 후 기본 명령 1회 이상 수행
- 권장 명령:
  - `ros2 launch jetrover_arm_moveit moveit_demo.launch.py use_fake_joint_states:=false`
- 기대 결과:
  - planning 파이프라인 기동 성공
  - follow_joint_trajectory 액션 경로 정상

## Case 3: 음성 파이프라인 기동 회귀
- 문제: wake/vad/stt/intent/tts 연결 실패
- 조건: voice pipeline launch 후 핵심 토픽 확인
- 권장 명령:
  - `ros2 launch tts_cpp voice_pipeline_with_tts.launch.py`
  - `ros2 topic list`
- 기대 결과:
  - 주요 노드 기동
  - 관련 토픽 확인 가능

## 실행/기록 규칙
- 각 케이스는 `명령`, `실행 시각`, `성공/실패`, `로그 경로`를 기록한다.
- 실패 시 원인 가설과 재현 조건을 함께 갱신한다.

# rover_ws Command Guide (Updated)

이 문서는 `/home/ubuntu/rover_ws`에서 바로 실행 가능한 명령을 최신 상태로 정리한 문서입니다.

주의:
- 항상 `~/rover_ws` 루트에서 빌드/실행하세요.
- `~/rover_ws/src`에서 `colcon build`하지 마세요.

---

## 0) 공통 준비

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/rover_ws/install/setup.bash
```

---

## 1) 빌드

### 전체 빌드
```bash
cd /home/ubuntu/rover_ws
colcon build --base-paths src
```

### 음성 파이프라인만 빌드
```bash
cd /home/ubuntu/rover_ws
colcon build --base-paths src --packages-select wake_vad_cpp stt_cpp intent_router_cpp llm_cpp tts_cpp
```

### 로봇팔/MoveIt 관련만 빌드
```bash
cd /home/ubuntu/rover_ws
colcon build --base-paths src --packages-select ros_robot_controller_msgs ros_robot_controller_cpp jetrover_arm_moveit
```

---

## 2) 음성 파이프라인 실행

### 클라우드 모드 (API 키 설정 시)
- STT  : moonshine-tiny-ko (로컬)
- LLM  : OpenAI → Groq → Gemini → Ollama (순서대로 폴백)
- TTS  : edge-tts(클라우드) → MeloTTS → espeak-ng (순서대로 폴백)
```bash
ros2 launch tts_cpp voice_pipeline_with_tts.launch.py
```

### 로컬 모드 (인터넷 없이 완전 로컬)
- STT  : moonshine-tiny-ko (로컬)
- LLM  : Ollama 직접 호출 (클라우드 API 스킵, ollama 실행 필요)
- TTS  : MeloTTS → espeak-ng (edge-tts 스킵)
```bash
ros2 launch tts_cpp voice_pipeline_local.launch.py
```

> Ollama 실행 확인: `curl http://127.0.0.1:11434/api/tags`

### 단계별 실행
```bash
ros2 launch stt_cpp wake_vad_with_stt.launch.py
ros2 launch intent_router_cpp voice_pipeline_with_router.launch.py
ros2 launch llm_cpp voice_pipeline_with_llm.launch.py
ros2 launch tts_cpp tts.launch.py
```

---

## 3) 로봇 하드웨어 컨트롤러 실행

```bash
ros2 launch ros_robot_controller_cpp ros_robot_controller_cpp.launch.py
```

확인:
```bash
ros2 node list | grep ros_robot_controller
```

---

## 4) MoveIt + 실로봇 동기화 실행

### 기본 실행
```bash
ros2 launch jetrover_arm_moveit moveit_demo.launch.py use_fake_joint_states:=false
### 터미널 1
ros2 launch ros_robot_controller_cpp ros_robot_controller_cpp.launch.py
### 터미널 2
ros2 launch tts_cpp voice_pipeline_with_tts.launch.py
```

### intent -> arm bridge 포함 실행 (기본값 true)
```bash
ros2 launch jetrover_arm_moveit moveit_demo.launch.py use_fake_joint_states:=false enable_intent_arm_bridge:=true
```

설명:
- `arm_servo_state_bridge.py`: 실서보 상태를 `/moveit_joint_states`로 브리지
- `follow_joint_trajectory_bridge.py`: MoveIt trajectory를 서보 명령으로 브리지
- `intent_arm_bridge_node`(C++): `/intent_router/robot_command`를 받아 팔 목표 자세로 변환해 액션 전송
- home 계열 키워드는 MoveIt 각도 목표가 아니라 pulse 고정값으로 직접 구동:
  - `servo_ids = [1,2,3,4,5,10]`
  - `target_home_pulses = [500,765,15,220,500,500]`
  - topic: `/ros_robot_controller/bus_servo/set_position`

---

## 5) intent 로봇팔 명령 테스트

아래 명령은 `moveit_demo.launch.py` 실행 중 테스트하세요.

```bash
ros2 topic pub --once /intent_router/category std_msgs/msg/String "{data: 'robot_command'}"
```

### 원위치 (home pulse 직행)
```bash
ros2 topic pub --once /intent_router/robot_command std_msgs/msg/String "{data: '{\"category\":\"robot_command\",\"command\":\"원위치\",\"chat_text\":\"\",\"needs_vision\":false}'}"
```

home 계열 키워드:
- `원위치`, `홈`, `home`, `초기`, `초기화`, `기본자세`, `리셋`, `reset`
- `팔 내려`, `팔을 내려`, `팔내려`, `내려`, `내려줘`, `내려와`, `팔 접어`, `접어줘`

동작:
- 위 키워드는 항상 `[500,765,15,220,500,500] pulse`로 이동

### 집기
```bash
ros2 topic pub --once /intent_router/robot_command std_msgs/msg/String "{data: '{\"category\":\"robot_command\",\"command\":\"집기\",\"chat_text\":\"\",\"needs_vision\":false}'}"
```

### 놓기
```bash
ros2 topic pub --once /intent_router/robot_command std_msgs/msg/String "{data: '{\"category\":\"robot_command\",\"command\":\"놓기\",\"chat_text\":\"\",\"needs_vision\":false}'}"
```

### 음성 예시 (intent_arm_bridge_node 매핑)
- 준비/올리기: `팔 들어`, `팔 올려`, `들어 올려`, `팔 펴`
- 집기: `집기`, `잡아줘`, `집어줘`, `집어 들어`
- 놓기: `놓기`, `놔줘`, `놓아줘`, `내려놔줘`, `풀어줘`

---

## 6) 자주 쓰는 상태 확인 명령

### 노드/토픽
```bash
ros2 node list
ros2 topic list
```

### 음성 파이프라인 토픽
```bash
ros2 topic echo /wake_vad/transcript
ros2 topic echo /intent_router/category
ros2 topic echo /intent_router/robot_command
ros2 topic echo /llm/provider
ros2 topic echo /tts/debug
```

### MoveIt/로봇팔 토픽
```bash
ros2 topic hz /moveit_joint_states
ros2 topic echo /moveit_joint_states --once
ros2 action list | grep follow_joint_trajectory
```

---

## 7) 로봇팔 UI 실행

### 터미널 A: ros controller
```bash
cd /home/ubuntu/rover_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/rover_ws/install/setup.bash
ros2 launch ros_robot_controller_cpp ros_robot_controller_cpp.launch.py
```

### 터미널 B: UDP bridge
```bash
cd /home/ubuntu/rover_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/rover_ws/install/setup.bash
ros2 run jetrover_web_bridge udp_servo_bridge_node
```

### 터미널 C: web ui
```bash
cd /home/ubuntu/rover_ws/src/robotic-arm-ui
export ROBOT_UDP_HOST=127.0.0.1
export ROBOT_UDP_PORT=9999
corepack pnpm install
corepack pnpm dev
```

브라우저:
- `http://localhost:3000`

---

## 8) 디버깅 유틸

### 마이크 장치 확인
```bash
arecord -l
```

### 시리얼 장치 확인
```bash
ls -l /dev/rrc /dev/ttyACM* /dev/ttyUSB*
```

### 장치 점유 확인
```bash
lsof /dev/rrc
```

### 실행 중 launch 종료
```bash
pkill -f "ros2 launch"
```

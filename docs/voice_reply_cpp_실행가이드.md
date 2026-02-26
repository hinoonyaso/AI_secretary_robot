# voice_reply_cpp 실행 방법

## 1) 환경 로드
```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
```

## 2) (처음 1회) 패키지 빌드
```bash
cd ~/rover_ws
colcon build --packages-select voice_reply_cpp
source ~/rover_ws/install/setup.bash
```

## 3) (선택) 음성 출력(TTS) 설치
`espeak-ng`가 없으면 텍스트 응답만 되고 스피커 출력은 안 됩니다.
```bash
sudo apt-get update
sudo apt-get install -y espeak-ng
```

## 4) 마이크 어레이 ASR 노드 먼저 실행
Hiwonder 튜토리얼 방식대로 실행해서 아래 토픽이 살아있어야 합니다.
- `/asr_node/voice_words` (`std_msgs/String`)

확인:
```bash
ros2 topic list | grep asr_node
```

## 5) C++ 응답 노드 실행
```bash
ros2 launch voice_reply_cpp voice_reply.launch.py
```

## 5-1) GPT 사용을 위한 API 키 설정
노드 실행 전에 같은 터미널에서 설정:
```bash
export OPENAI_API_KEY=\"여기에_본인_API_KEY\"
```

영구 적용(선택):
```bash
echo 'export OPENAI_API_KEY=\"여기에_본인_API_KEY\"' >> ~/.bashrc
source ~/.bashrc
```

## 6) 수동 테스트
다른 터미널에서:
```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 topic pub --once /asr_node/voice_words std_msgs/msg/String "{data: '안녕'}"
```

## 7) 응답 확인
```bash
ros2 topic echo /voice_reply/text
```

정상 예시:
- 입력: `안녕`
- 출력: `안녕하세요. 무엇을 도와드릴까요?`

GPT가 활성화되면 규칙기반 대신 GPT 답변이 우선 사용되고,
API 실패/타임아웃 시 자동으로 규칙기반 답변으로 폴백됩니다.

---

## 문제 발생 시 점검

### A. 패키지를 못 찾는 경우
```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 pkg list | grep voice_reply_cpp
```

### B. ASR 토픽이 없는 경우
ASR 노드가 먼저 실행되어야 합니다.
```bash
ros2 topic list | grep /asr_node/voice_words
```

### C. 소리는 안 나고 로그에 `espeak-ng: not found` 뜨는 경우
```bash
sudo apt-get install -y espeak-ng
```

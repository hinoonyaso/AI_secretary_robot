# GPT Voice Reply 사용법

## 1) 실행
```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch voice_reply_cpp voice_reply.launch.py
```

## 2) 테스트 (다른 터미널)
```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 topic pub --once /asr_node/voice_words std_msgs/msg/String "{data: '안녕, 오늘 할 일 추천해줘'}"
```

## 3) 응답 확인
```bash
ros2 topic echo /voice_reply/text
```

## 4) 실사용
- Hiwonder 마이크 어레이 ASR 노드가 `/asr_node/voice_words`를 발행해야 합니다.
- voice_reply_cpp는 해당 텍스트를 받아 GPT로 답변 생성 후 TTS로 출력합니다.

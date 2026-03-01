# 현재 상태

업데이트 날짜: 2026-03-01

## 사용 스택
- ROS2 Humble (Host)
- Docker 기반 AI 파이프라인 (Brain)
- Jetson 환경 GPU 추론
- MoveIt2 + TRAC-IK
- Serial/UART 기반 모터/서보 제어
- 음성 파이프라인: Wake/VAD + STT + Intent + LLM + TTS

## 현재 문제
- Serial 통신 간헐적 실패 가능성
- launch 2~3회 반복 시 패킷 드랍/초기화 이슈 가능성
- 하드웨어 의존 노드의 재기동 안정성 점검 필요

## 제약
- STM/보드 펌웨어 수정 불가(가정)
- ROS2 패키지 구조 유지
- 기존 토픽/서비스 인터페이스 호환성 유지

## 현재 실행/검증 기준 문서
- `docs/command.md`: 빌드/실행 표준 명령
- `tests/regression_cases.md`: 회귀 테스트 기준 케이스

## 현재 우선순위
1. 재기동/반복 launch 안정성 확보
2. serial 통신 안정성 검증 자동화
3. 음성 파이프라인과 로봇 제어 경로의 회귀 방지

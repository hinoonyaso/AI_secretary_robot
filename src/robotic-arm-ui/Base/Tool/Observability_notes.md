# Constraint Observability Notes

## Logged Fields
- `ts`, `ts_ms`
- `frameType`, `constraintType`, `constraintAxis`, `axisVecOrNormalBase`
- `x_cmd` (`pose` or `twist`)
- `x_meas` (`pos`, `quat`, `source`)
- `residual` (`active`, `plane_n_dot_dp`, `line_orthogonal_error`, `point_position_error`)
- `T_base_tool`, `T_base_user` (when available)
- `latency_ms`, `loop_rate_hz`

## Missing / Approximated Fields
- `x_meas`:
  - 현재 실제 센서 피드백 스트림이 연결되어 있지 않음.
  - 따라서 `x_meas.source = "estimated_fk"`로 표시하며, IK 결과 서보값의 FK 추정치를 사용.
- `loop_rate_hz`:
  - 별도 주기 제어 루프가 아닌 이벤트(조그 명령) 기반.
  - 연속 샘플 간 시간차로 유사 loop rate를 계산.
- `T_base_user`:
  - User frame 미설정 시 필드 생략 가능.
- `latency_ms`:
  - 현재 경로에서는 제약 계산/적용 함수 구간의 소프트웨어 지연만 포함.
  - 실제 하드웨어 actuator 응답 지연은 미포함.

## Enable / Disable
- 기본값: OFF (production-safe)
- 활성화 경로:
  - URL: `?constraintLog=1`
  - localStorage: `constraint_logger_enabled=1`
  - env: `NEXT_PUBLIC_CONSTRAINT_LOGGER=1`


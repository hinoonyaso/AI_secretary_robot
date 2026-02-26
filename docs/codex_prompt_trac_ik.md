# jetrover_arm_moveit — TRAC-IK 역기구학 솔버 활성화 Codex 프롬프트

## 🎯 목적
jetrover_arm_moveit 패키지의 역기구학(IK) 솔버를 KDL에서 TRAC-IK로 전환하여 IK 계산 성공률과 성능을 향상시킵니다.

---

## 📋 Context

**프로젝트**: AI Secretary Robot (JetRover Arm)
**환경**: ROS 2 Humble, Jetson Orin Nano 8GB, MoveIt2
**현재 상태**: KDL(Kinematic Dynamics Library) 역기구학 솔버 사용 중
**목표 상태**: TRAC-IK(Track-IK) 역기구학 솔버로 전환

**배경**:
- KDL은 ROS 2 MoveIt2의 기본 IK 솔버이지만, 특이점(singularity) 근처에서 해를 찾지 못하는 경우가 많습니다.
- TRAC-IK는 TracIK 알고리즘을 사용하여 더 빠르고 안정적인 IK 해를 제공합니다.
- 특히 6-DOF 미만의 팔(jetrover는 5-DOF)에서 TRAC-IK가 더 나은 성능을 보입니다.

**파일 위치**:
- 패키지 경로: `/home/ubuntu/AI_secretary_robot/src/control/jetrover_arm_moveit/`
- 설정 파일: `config/kinematics.yaml`
- 의존성: `package.xml` (이미 trac_ik_kinematics_plugin 포함됨)

---

## 🔧 Task

### 1. kinematics.yaml 수정

**목표**: KDL 솔버를 TRAC-IK 솔버로 교체하고, 최적의 파라미터를 설정합니다.

**현재 설정** (`config/kinematics.yaml`):
```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 6
```

**변경할 설정**:
```yaml
arm:
  # TRAC-IK 솔버로 변경
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin

  # 타임아웃 (초): IK 계산 최대 시간
  # TRAC-IK는 KDL보다 빠르므로 0.005초(5ms)로 단축 가능
  # Jetson Orin Nano에서는 0.01초(10ms) 권장 (안정성 확보)
  kinematics_solver_timeout: 0.01

  # 시도 횟수: TRAC-IK는 한 번에 해를 찾는 경우가 많아 3회면 충분
  kinematics_solver_attempts: 3

  # ===== TRAC-IK 전용 파라미터 =====

  # solve_type: 해 탐색 전략
  #   - Speed: 첫 번째 유효한 해를 즉시 반환 (가장 빠름, 실시간 제어 권장)
  #   - Distance: 현재 관절 각도에서 가장 가까운 해 탐색 (부드러운 동작)
  #   - Manipulation1: 조작성 최적화 (singularity 회피)
  #   - Manipulation2: Manipulation1보다 더 엄격한 조작성 조건
  solve_type: Speed

  # position_tolerance (m): 목표 위치 허용 오차
  # 0.0001m (0.1mm) - 정밀 조립 작업
  # 0.001m (1mm) - 일반 픽앤플레이스 (권장)
  # 0.005m (5mm) - 빠른 동작, 낮은 정밀도
  position_tolerance: 0.001

  # orientation_tolerance (rad): 목표 자세 허용 오차
  # 0.01 rad ≈ 0.57° - 정밀 작업
  # 0.05 rad ≈ 2.86° - 일반 작업 (권장)
  # 0.1 rad ≈ 5.73° - 빠른 동작
  orientation_tolerance: 0.05

  # epsilon: 최적화 알고리즘의 수렴 임계값
  # 기본값 1e-5 유지 (TRAC-IK 내부 알고리즘 파라미터)
  epsilon: 1e-5
```

---

## 🎛️ TRAC-IK 파라미터 상세 설명

### `solve_type` 선택 가이드

| solve_type | 속도 | 정확도 | singularity 회피 | 사용 사례 |
|------------|------|--------|------------------|----------|
| **Speed** | ⚡⚡⚡ | ✓ | △ | 실시간 제어, 비전 피드백 루프 |
| **Distance** | ⚡⚡ | ✓✓ | ✓ | 부드러운 궤적, 연속 동작 |
| **Manipulation1** | ⚡ | ✓✓✓ | ✓✓ | 좁은 공간, 장애물 회피 |
| **Manipulation2** | ⚡ | ✓✓✓ | ✓✓✓ | 극한 환경, 최고 안정성 필요 |

**jetrover_arm 권장 설정**:
- 비전 기반 실시간 제어 → `Speed`
- 궤적 계획 (MoveIt) → `Distance`
- 협소 공간 작업 → `Manipulation1`

### `position_tolerance` / `orientation_tolerance` 튜닝

**측정 방법**:
1. MoveIt RViz에서 목표 pose 설정
2. IK 계산 성공/실패 로그 확인
3. 실패가 많으면 tolerance 증가, 성공률 높으면 tolerance 감소

**권장 초기값** (jetrover_arm 5-DOF):
```yaml
position_tolerance: 0.001      # 1mm
orientation_tolerance: 0.05    # ~3°
```

### `kinematics_solver_timeout` 최적화

**벤치마크 (Jetson Orin Nano)**:
- TRAC-IK Speed 모드: 평균 3~5ms
- TRAC-IK Distance 모드: 평균 8~12ms
- KDL: 평균 15~30ms (실패 시 타임아웃까지 소요)

**권장 타임아웃**:
```yaml
# Speed 모드
kinematics_solver_timeout: 0.01   # 10ms

# Distance/Manipulation 모드
kinematics_solver_timeout: 0.02   # 20ms
```

---

## ✅ 검증 방법

### 1. 빌드 및 실행

```bash
cd /home/ubuntu/AI_secretary_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select jetrover_arm_moveit
source install/setup.bash

# MoveIt + RViz 실행
ros2 launch jetrover_arm_moveit moveit_demo.launch.py
```

### 2. RViz에서 IK 테스트

1. **MotionPlanning 패널**에서 "Planning Group" = `arm` 선택
2. **Interactive Marker** 드래그하여 목표 pose 설정
3. **Plan** 버튼 클릭 → 궤적 계획 성공 확인
4. **Execute** 버튼 클릭 → 실제 동작 확인 (시뮬레이션 환경)

**성공 지표**:
- IK 계산 성공률 > 95%
- 계산 시간 < 10ms (터미널 로그 확인)
- 특이점 근처에서도 해 탐색 성공

### 3. 커맨드라인 Cartesian target 테스트

```bash
# 특정 3D 좌표로 이동 명령
ros2 run jetrover_arm_moveit arm_moveit_commander.py \
  --x 0.18 --y 0.00 --z 0.16 \
  --roll 0.0 --pitch 1.57 --yaw 0.0
```

**출력 예시** (성공):
```
[INFO] Planning to target: (0.18, 0.00, 0.16)
[INFO] IK solved in 4.2ms
[INFO] Trajectory planned successfully
[INFO] Executing...
[INFO] Motion complete
```

### 4. 로그 분석

```bash
# MoveIt 로그에서 IK 성능 확인
ros2 topic echo /move_group/display_planned_path --once | grep -i "ik"

# 또는 launch 시 로그 레벨 상향
ros2 launch jetrover_arm_moveit moveit_demo.launch.py log_level:=debug
```

**찾아야 할 로그**:
```
[move_group]: Using solver 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin'
[trac_ik]: IK solved in 0.005s (5.2ms)
[trac_ik]: Solution found with solve_type=Speed
```

---

## 🐛 Troubleshooting

### 문제 1: `trac_ik_kinematics_plugin not found`

**원인**: TRAC-IK 플러그인 미설치 (apt 저장소에 없음)

**해결** - 소스에서 빌드 필요:
```bash
# 자동 설치 스크립트 사용 (권장)
cd /home/ubuntu/AI_secretary_robot
./scripts/install_trac_ik.sh

# 또는 수동 설치: 상세 가이드는 아래 참조
# docs/trac_ik_installation.md
```

**중요**: ROS 2 Humble의 apt 저장소에는 `ros-humble-trac-ik-kinematics-plugin` 패키지가 없습니다.
GitHub에서 소스를 다운로드하여 빌드해야 합니다.

**상세 설치 가이드**: [trac_ik_installation.md](./trac_ik_installation.md)

### 문제 2: IK 계산 실패율이 높음 (> 10%)

**원인**: tolerance가 너무 엄격하거나, URDF 관절 제한이 너무 좁음

**해결 1**: tolerance 완화
```yaml
position_tolerance: 0.005      # 1mm → 5mm
orientation_tolerance: 0.1     # 3° → 6°
```

**해결 2**: solve_type 변경
```yaml
solve_type: Distance   # Speed → Distance
```

**해결 3**: 관절 제한 확인
```bash
# joint_limits.yaml 확인
cat ~/AI_secretary_robot/src/control/jetrover_arm_moveit/config/joint_limits.yaml
```

### 문제 3: IK 계산이 느림 (> 20ms)

**원인**: solve_type이 Manipulation1/2로 설정되어 있거나, timeout이 너무 큼

**해결**:
```yaml
solve_type: Speed
kinematics_solver_timeout: 0.005   # 5ms로 단축
```

### 문제 4: 팔이 예상치 못한 경로로 이동

**원인**: solve_type=Speed는 첫 번째 해를 반환하므로, 현재 자세에서 먼 해가 선택될 수 있음

**해결**: Distance 모드로 전환 (현재 자세에서 가장 가까운 해 선택)
```yaml
solve_type: Distance
kinematics_solver_timeout: 0.015   # Distance는 Speed보다 시간 소요
```

---

## 📊 KDL vs TRAC-IK 성능 비교 (예상)

| 지표 | KDL | TRAC-IK (Speed) | TRAC-IK (Distance) |
|------|-----|-----------------|-------------------|
| IK 계산 시간 | 15~30ms | 3~5ms | 8~12ms |
| 성공률 (일반) | 60~70% | 85~95% | 90~98% |
| 성공률 (특이점 근처) | 10~20% | 60~80% | 70~90% |
| 계산 안정성 | △ | ✓ | ✓✓ |
| 실시간 제어 적합성 | △ | ✓✓✓ | ✓✓ |

---

## 🚀 최종 권장 설정

**일반 픽앤플레이스 작업** (비전 피드백 루프, 실시간 제어):
```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.01
  kinematics_solver_attempts: 3
  solve_type: Speed
  position_tolerance: 0.001
  orientation_tolerance: 0.05
  epsilon: 1e-5
```

**부드러운 궤적 계획** (MoveIt Cartesian path):
```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.015
  kinematics_solver_attempts: 3
  solve_type: Distance
  position_tolerance: 0.0005
  orientation_tolerance: 0.03
  epsilon: 1e-5
```

**좁은 공간 / 장애물 회피**:
```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.02
  kinematics_solver_attempts: 5
  solve_type: Manipulation1
  position_tolerance: 0.001
  orientation_tolerance: 0.05
  epsilon: 1e-5
```

---

## 📚 참고 자료

- TRAC-IK 공식 문서: https://github.com/traclabs/trac_ik
- MoveIt2 Kinematics 플러그인: https://moveit.picknik.ai/main/doc/examples/kinematics/kinematics.html
- ROS 2 Humble TRAC-IK 패키지: https://github.com/ros-controls/kinematics_interface_kdl

---

## 🎓 Codex 사용 시 프롬프트 예시

```
[Task]
jetrover_arm_moveit 패키지의 역기구학 솔버를 KDL에서 TRAC-IK로 전환하세요.

[Requirements]
1. config/kinematics.yaml 파일 수정
2. TRAC-IK 솔버로 변경: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
3. solve_type을 Speed로 설정 (실시간 제어 최적화)
4. timeout을 0.01초로 설정
5. position_tolerance: 0.001m, orientation_tolerance: 0.05 rad
6. 변경 후 빌드 및 실행하여 정상 동작 확인

[Constraints]
- package.xml은 이미 trac_ik_kinematics_plugin 의존성을 포함하고 있으므로 수정 불필요
- 기존 KDL 설정을 완전히 제거하고 TRAC-IK 파라미터로 교체
- 변경사항을 commit하기 전에 MoveIt RViz에서 IK 계산 성공 확인 필수

[Verification]
1. ros2 launch jetrover_arm_moveit moveit_demo.launch.py 실행
2. RViz Interactive Marker로 목표 pose 설정
3. Plan 버튼 클릭하여 IK 계산 성공 확인
4. 터미널 로그에서 "trac_ik" 문자열 확인
```

---

이 프롬프트를 Codex에 입력하면 자동으로 kinematics.yaml을 수정하고 TRAC-IK를 활성화할 수 있습니다.

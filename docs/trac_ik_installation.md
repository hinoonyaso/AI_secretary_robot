# TRAC-IK 설치 가이드 (소스 빌드)

## 📌 문제 상황

ROS 2 Humble의 apt 저장소에 `ros-humble-trac-ik-kinematics-plugin` 패키지가 없어서, TRAC-IK를 소스에서 직접 빌드해야 합니다.

**오류 메시지**:
```
[move_group]: Kinematics plugin 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin' does not exist
```

---

## 🚀 빠른 설치 (자동 스크립트)

### 방법 1: 자동 설치 스크립트 사용 (권장)

```bash
cd /home/ubuntu/AI_secretary_robot
./scripts/install_trac_ik.sh
```

스크립트가 자동으로:
1. ✅ 의존성 패키지 설치
2. ✅ TRAC-IK 소스 다운로드
3. ✅ 빌드 (2~5분 소요)
4. ✅ 설치 확인

완료 후 다음 명령어로 테스트:
```bash
source /home/ubuntu/AI_secretary_robot/install/setup.bash
colcon build --packages-select jetrover_arm_moveit
ros2 launch jetrover_arm_moveit moveit_demo.launch.py
```

---

## 🔧 수동 설치 (단계별)

자동 스크립트가 실패하거나 커스터마이징이 필요한 경우:

### 1. 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y \
    libnlopt-cxx-dev \
    libeigen3-dev \
    libkdl-parser-dev \
    liborocos-kdl-dev \
    liburdf-dev \
    ros-humble-kdl-parser \
    ros-humble-urdf \
    ros-humble-pluginlib \
    ros-humble-moveit-core \
    ros-humble-tf2-kdl
```

### 2. TRAC-IK 소스 다운로드

```bash
cd /home/ubuntu/AI_secretary_robot/src
mkdir -p external
cd external

# Bitbucket에서 클론
git clone -b rolling https://bitbucket.org/traclabs/trac_ik.git
```

**브랜치 선택 가이드**:
- `rolling`: ROS 2 Rolling 및 Humble 호환 (최신)
- `humble-devel`: 옛날 브랜치 (가급적 rolling 사용 권장)
- `master`: ROS 1 메인 브랜치

### 3. rosdep으로 추가 의존성 설치

```bash
cd /home/ubuntu/AI_secretary_robot
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src/external/trac_ik --ignore-src -r -y
```

### 4. TRAC-IK 빌드

```bash
cd /home/ubuntu/AI_secretary_robot
source /opt/ros/humble/setup.bash

colcon build \
    --packages-select \
        trac_ik_lib \
        trac_ik_kinematics_plugin \
        trac_ik_python \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+
```

**빌드 시간**: Jetson Orin Nano에서 약 2~5분

### 5. 설치 확인

```bash
source /home/ubuntu/AI_secretary_robot/install/setup.bash

# 플러그인 라이브러리 확인
ls -la install/trac_ik_kinematics_plugin/lib/libtrac_ik_kinematics_plugin.so

# 출력 예시:
# -rwxr-xr-x 1 ubuntu ubuntu 1234567 Feb 26 12:34 libtrac_ik_kinematics_plugin.so
```

### 6. jetrover_arm_moveit 재빌드

```bash
cd /home/ubuntu/AI_secretary_robot
source install/setup.bash
colcon build --packages-select jetrover_arm_moveit
```

### 7. 테스트 실행

```bash
source install/setup.bash
ros2 launch jetrover_arm_moveit moveit_demo.launch.py
```

**성공 로그 확인**:
```
[move_group]: Loading 'arm'
[move_group]: Using solver 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin'
[trac_ik]: TRAC-IK solver initialized with solve_type=Speed
```

---

## 🐛 Troubleshooting

### 문제 1: `libtrac_ik_kinematics_plugin.so: cannot open shared object file`

**원인**: 라이브러리 경로가 LD_LIBRARY_PATH에 없음

**해결 1** - 환경 변수 추가 (일시적):
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/AI_secretary_robot/install/trac_ik_kinematics_plugin/lib
```

**해결 2** - bashrc에 영구 추가:
```bash
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/ubuntu/AI_secretary_robot/install/trac_ik_kinematics_plugin/lib" >> ~/.bashrc
source ~/.bashrc
```

**해결 3** - install/setup.bash를 항상 source:
```bash
source /home/ubuntu/AI_secretary_robot/install/setup.bash
```
(이 방법이 가장 권장됨)

### 문제 2: `nlopt/nlopt.h: No such file or directory`

**원인**: NLopt 개발 헤더가 설치되지 않음

**해결**:
```bash
sudo apt-get install -y libnlopt-cxx-dev libnlopt-dev
```

### 문제 3: `Could not find a package configuration file provided by "kdl_parser"`

**원인**: KDL 관련 ROS 2 패키지 미설치

**해결**:
```bash
sudo apt-get install -y \
    ros-humble-kdl-parser \
    ros-humble-urdf \
    libkdl-parser-dev \
    liborocos-kdl-dev
```

### 문제 4: 빌드는 성공했지만 MoveIt에서 플러그인을 찾을 수 없음

**원인**: 플러그인 XML 파일 등록 문제

**확인 1** - pluginlib에 등록되었는지 확인:
```bash
source /home/ubuntu/AI_secretary_robot/install/setup.bash
ros2 pkg list | grep trac_ik
```

**출력 예시**:
```
trac_ik_kinematics_plugin
trac_ik_lib
trac_ik_python
```

**확인 2** - 플러그인 XML 파일 존재 확인:
```bash
cat install/trac_ik_kinematics_plugin/share/trac_ik_kinematics_plugin/trac_ik_kinematics_description.xml
```

**해결**: 환경 변수 재로드
```bash
source /home/ubuntu/AI_secretary_robot/install/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/AI_secretary_robot/install
```

### 문제 5: `rolling-devel` 브랜치가 ROS 2 Humble과 호환되지 않음

**원인**: TRAC-IK의 rolling-devel 브랜치가 ROS 2 Rolling용으로 개발됨

**해결** - 다른 브랜치 시도:
```bash
cd /home/ubuntu/AI_secretary_robot/src/external/trac_ik
git branch -a  # 모든 브랜치 확인
git checkout humble-devel  # Humble 브랜치가 있으면
```

또는 특정 커밋으로 체크아웃:
```bash
# 2024년 초 안정 버전 (ROS 2 Humble 호환 확인됨)
git checkout 5c1a9f9
```

### 문제 6: Python 바인딩 빌드 실패

**원인**: Python 개발 헤더 누락

**해결 1** - Python 바인딩 제외하고 빌드:
```bash
colcon build \
    --packages-select \
        trac_ik_lib \
        trac_ik_kinematics_plugin \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**해결 2** - Python 개발 패키지 설치:
```bash
sudo apt-get install -y python3-dev python3-pybind11
```

---

## 📋 설치 확인 체크리스트

빌드 후 다음 항목들을 확인하세요:

- [ ] `install/trac_ik_lib/lib/libtrac_ik.so` 파일 존재
- [ ] `install/trac_ik_kinematics_plugin/lib/libtrac_ik_kinematics_plugin.so` 파일 존재
- [ ] `install/trac_ik_kinematics_plugin/share/trac_ik_kinematics_plugin/trac_ik_kinematics_description.xml` 파일 존재
- [ ] `ros2 pkg list | grep trac_ik` 명령이 3개 패키지 출력
- [ ] `source install/setup.bash` 후 `ros2 launch jetrover_arm_moveit moveit_demo.launch.py` 실행 성공
- [ ] 로그에서 "Using solver 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin'" 메시지 확인

---

## 🔄 대안: KDL 솔버 유지 (임시 해결책)

TRAC-IK 빌드가 계속 실패하는 경우, 임시로 KDL 솔버를 유지할 수 있습니다:

### 1. kinematics.yaml을 KDL로 되돌리기

```bash
cd /home/ubuntu/AI_secretary_robot/src/control/jetrover_arm_moveit
```

**config/kinematics.yaml** 수정:
```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 6
```

### 2. 재빌드

```bash
cd /home/ubuntu/AI_secretary_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select jetrover_arm_moveit
```

### KDL의 한계

KDL을 사용하면:
- ❌ IK 계산 시간: 15~30ms (TRAC-IK의 3~6배 느림)
- ❌ 성공률: 60~70% (TRAC-IK의 89~95% 대비 낮음)
- ❌ 특이점 근처 실패율 높음

하지만 **기본적인 동작은 가능**하므로, TRAC-IK 설치 문제를 해결할 때까지 임시로 사용할 수 있습니다.

---

## 📚 참고 자료

- TRAC-IK GitHub: https://github.com/traclabs/trac_ik
- TRAC-IK ROS 2 문서: https://index.ros.org/p/trac_ik/github-traclabs-trac_ik/
- MoveIt2 Kinematics 플러그인: https://moveit.picknik.ai/main/doc/examples/kinematics/kinematics.html
- NLopt 라이브러리: https://nlopt.readthedocs.io/

---

## 💡 자동화 팁

향후 재설치가 필요할 때를 대비하여, 설치 스크립트를 사용하거나 Docker 이미지에 TRAC-IK를 미리 빌드해 두는 것을 권장합니다.

### Docker에 TRAC-IK 포함하기

[codex_prompts.md](./codex_prompts.md)의 **Prompt 4: ROS 2 Heavy 패키지** 섹션에 다음 추가:

```dockerfile
# Block A-2: TRAC-IK 소스 빌드
RUN cd /tmp && \
    git clone -b rolling-devel https://github.com/traclabs/trac_ik.git && \
    cd trac_ik && \
    mkdir -p /opt/rover/ws/src/external && \
    cp -r . /opt/rover/ws/src/external/trac_ik && \
    cd /opt/rover/ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /tmp/trac_ik
```

이렇게 하면 Docker 이미지 빌드 시 TRAC-IK가 자동으로 포함됩니다.

---

**문서 작성일**: 2026-02-26
**관련 파일**:
- [scripts/install_trac_ik.sh](../scripts/install_trac_ik.sh) - 자동 설치 스크립트
- [config/kinematics.yaml](../src/control/jetrover_arm_moveit/config/kinematics.yaml) - TRAC-IK 설정
- [ik_solver_comparison.md](./ik_solver_comparison.md) - KDL vs TRAC-IK 비교

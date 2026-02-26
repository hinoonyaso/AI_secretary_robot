#!/bin/bash
# TRAC-IK 소스 빌드 및 설치 스크립트
# ROS 2 Humble용 TRAC-IK kinematics plugin을 소스에서 빌드합니다.

set -e  # 에러 발생 시 스크립트 중단

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}TRAC-IK 소스 빌드 및 설치 스크립트${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 1. ROS 2 환경 확인
echo -e "${YELLOW}[1/6] ROS 2 환경 확인 중...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS 2 환경이 설정되지 않았습니다. sourcing...${NC}"
    source /opt/ros/humble/setup.bash
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo -e "${RED}ERROR: ROS 2 Humble이 필요합니다. 현재: $ROS_DISTRO${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS 2 Humble 환경 확인 완료${NC}"
echo ""

# 2. 의존성 설치
echo -e "${YELLOW}[2/6] 의존성 패키지 설치 중...${NC}"
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

echo -e "${GREEN}✓ 의존성 설치 완료${NC}"
echo ""

# 3. workspace 생성 (이미 있으면 스킵)
WORKSPACE_DIR="/home/ubuntu/AI_secretary_robot"
TRAC_IK_SRC_DIR="$WORKSPACE_DIR/src/external"

echo -e "${YELLOW}[3/6] TRAC-IK 소스 다운로드 중...${NC}"

# external 디렉토리 생성
mkdir -p "$TRAC_IK_SRC_DIR"
cd "$TRAC_IK_SRC_DIR"

# 이미 클론되어 있으면 삭제
if [ -d "trac_ik" ]; then
    echo -e "${YELLOW}기존 trac_ik 디렉토리 삭제 중...${NC}"
    rm -rf trac_ik
fi

# TRAC-IK 저장소 클론
echo -e "${YELLOW}Bitbucket에서 TRAC-IK 클론 중...${NC}"
git clone -b rolling https://bitbucket.org/traclabs/trac_ik.git

# ROS 2 Humble 호환 브랜치 확인
cd trac_ik
echo -e "${YELLOW}사용 가능한 브랜치 확인 중...${NC}"
git branch -a

# rolling 브랜치가 ROS 2 Humble과 호환되는지 확인
# 필요시 다른 브랜치로 체크아웃 가능
echo -e "${GREEN}✓ TRAC-IK 소스 다운로드 완료${NC}"
echo ""

# 4. rosdep으로 추가 의존성 설치
echo -e "${YELLOW}[4/6] rosdep으로 추가 의존성 설치 중...${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash

# rosdep 업데이트
rosdep update 2>/dev/null || true

# TRAC-IK 의존성 설치
rosdep install --from-paths src/external/trac_ik --ignore-src -r -y || {
    echo -e "${YELLOW}Warning: 일부 rosdep 패키지를 찾을 수 없습니다. 계속 진행합니다...${NC}"
}

echo -e "${GREEN}✓ rosdep 의존성 설치 완료${NC}"
echo ""

# 5. TRAC-IK 빌드
echo -e "${YELLOW}[5/6] TRAC-IK 빌드 중... (2~5분 소요)${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash

# TRAC-IK만 선택적으로 빌드
colcon build \
    --packages-select \
        trac_ik_lib \
        trac_ik_kinematics_plugin \
        trac_ik_python \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

if [ $? -ne 0 ]; then
    echo -e "${RED}ERROR: TRAC-IK 빌드 실패${NC}"
    exit 1
fi

echo -e "${GREEN}✓ TRAC-IK 빌드 완료${NC}"
echo ""

# 6. 설치 확인
echo -e "${YELLOW}[6/6] 설치 확인 중...${NC}"
source "$WORKSPACE_DIR/install/setup.bash"

# 플러그인 라이브러리 확인
PLUGIN_LIB="$WORKSPACE_DIR/install/trac_ik_kinematics_plugin/lib/libtrac_ik_kinematics_plugin.so"
if [ -f "$PLUGIN_LIB" ]; then
    echo -e "${GREEN}✓ TRAC-IK 플러그인 라이브러리 발견: $PLUGIN_LIB${NC}"
else
    echo -e "${RED}ERROR: TRAC-IK 플러그인 라이브러리를 찾을 수 없습니다${NC}"
    exit 1
fi

# Python 바인딩 확인 (선택적)
if python3 -c "import trac_ik_python" 2>/dev/null; then
    echo -e "${GREEN}✓ TRAC-IK Python 바인딩 설치 완료${NC}"
else
    echo -e "${YELLOW}Warning: TRAC-IK Python 바인딩이 설치되지 않았습니다 (선택적)${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}TRAC-IK 설치 완료!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}다음 단계:${NC}"
echo ""
echo -e "1. 터미널을 새로 열거나 다음 명령어를 실행하세요:"
echo -e "   ${YELLOW}source /home/ubuntu/AI_secretary_robot/install/setup.bash${NC}"
echo ""
echo -e "2. jetrover_arm_moveit 재빌드:"
echo -e "   ${YELLOW}cd /home/ubuntu/AI_secretary_robot${NC}"
echo -e "   ${YELLOW}colcon build --packages-select jetrover_arm_moveit${NC}"
echo ""
echo -e "3. MoveIt 실행:"
echo -e "   ${YELLOW}ros2 launch jetrover_arm_moveit moveit_demo.launch.py${NC}"
echo ""
echo -e "4. 로그에서 다음 메시지를 확인하세요:"
echo -e "   ${GREEN}[move_group]: Using solver 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin'${NC}"
echo ""
echo -e "${BLUE}Troubleshooting:${NC}"
echo -e "플러그인을 찾을 수 없다는 오류가 나오면:"
echo -e "   ${YELLOW}export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/ubuntu/AI_secretary_robot/install/trac_ik_kinematics_plugin/lib${NC}"
echo ""

# JetRover Arm MoveIt2 (Phase 3)

This package provides a practical MoveIt2 integration baseline for JetRover arm:
- TRAC-IK kinematics config (`config/kinematics.yaml`)
- MoveIt launch (`launch/moveit_demo.launch.py`)
- RViz planning/execution test setup
- Cartesian target test script (`scripts/arm_moveit_commander.py`)

## 1) Install MoveIt2 + TRAC-IK

### MoveIt2 설치
```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-fake-controller-manager \
  ros-humble-xacro
```

### TRAC-IK 설치 (소스 빌드 필요)

**중요**: `ros-humble-trac-ik-kinematics-plugin`은 apt 저장소에 없습니다.
소스에서 빌드해야 합니다.

**자동 설치 (권장)**:
```bash
cd /home/ubuntu/AI_secretary_robot
./scripts/install_trac_ik.sh
```

**수동 설치**: [docs/trac_ik_installation.md](../../../docs/trac_ik_installation.md) 참조

## 2) Build

```bash
cd ~/rover_ws/src
source /opt/ros/humble/setup.bash
colcon build --packages-select jetrover_arm_moveit
source install/setup.bash
```

## 3) Setup Assistant workflow (recommended)

Use this package as baseline, then refine with Setup Assistant:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Inside Setup Assistant:
1. Load URDF/Xacro: `~/rover_ws/src/jetrover_arm_moveit/urdf/arm.urdf.xacro`
2. Create planning group `arm` (`joint2`, `joint3`, `joint4`, `joint5`)
3. End-effector link: `end_effector_link`
4. Generate collision matrix
5. Save/update config package (you can overwrite files in this package)

## 4) Run MoveIt + RViz

```bash
cd ~/rover_ws/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jetrover_arm_moveit moveit_demo.launch.py
```

Optional xacro path override:

```bash
ros2 launch jetrover_arm_moveit moveit_demo.launch.py xacro_path:=/home/ubuntu/rover_ws/src/jetrover_arm_moveit/urdf/arm.urdf.xacro
```

## 5) Kinematics solver switch (KDL -> TRAC-IK)

Already configured in `config/kinematics.yaml`:

```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
```

## 6) Cartesian target test (`arm_moveit_commander.py`)

With move_group running:

```bash
cd ~/rover_ws/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetrover_arm_moveit arm_moveit_commander \
  --x 0.18 --y 0.00 --z 0.16 \
  --roll 0.0 --pitch 1.57 --yaw 0.0
```

If the executable name above is not found in your environment, use:

```bash
ros2 run jetrover_arm_moveit arm_moveit_commander.py \
  --x 0.18 --y 0.00 --z 0.16 \
  --roll 0.0 --pitch 1.57 --yaw 0.0
```

## 7) Collision avoidance check

1. In RViz MotionPlanning panel, add a box/cylinder collision object
2. Plan to a target behind/through the object
3. Verify planner produces collision-free trajectory or fails safely

## Notes

- Current SRDF is a hand-authored baseline (`config/jetrover_arm.srdf`).
- For production, regenerate SRDF + collision matrix using Setup Assistant.
- URDF/Xacro now references `package://jetrover_arm_moveit/meshes/...`.
- If `ros2 launch jetrover_arm_moveit ...` says package not found, install MoveIt/TRAC-IK first, then rebuild and re-source:
  - `source /opt/ros/humble/setup.bash`
  - `cd ~/rover_ws/src && colcon build --packages-select jetrover_arm_moveit`
  - `source ~/rover_ws/src/install/setup.bash`

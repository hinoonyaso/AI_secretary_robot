# JetRover "Rover" — Host 미구현 부분 Codex 프롬프트 모음

> **실행 순서**: Prompt H1 → H2 → H3 → H4 → H5 → H6 (순서대로 진행)
> **목표**: Host (Ubuntu 22.04 네이티브) 환경의 Nav2, SLAM, 통합 시스템 구축
> **환경**: Jetson Orin Nano 8GB / Ubuntu 22.04 / ROS2 Humble / JetPack 6.0
> **역할 분담**: Host = 로보틱스 제어 / Brain (Docker) = AI 추론

---

## 📍 Prompt H1: Nav2 설치 및 설정

```
[Context]
- Host에는 ROS2 Humble이 이미 설치되어 있음
- 하드웨어: 4WD 메카넘 휠, RPLIDAR A1 (360° LiDAR), IMU (MPU6050)
- 목표: Nav2를 설치하고 메카넘 휠 + LiDAR 기반 자율 주행 설정
- Output: Nav2 설정 파일 및 launch 파일 생성

[Critical Requirements]
⚠️ 메카넘 휠은 전방향 이동이 가능하므로, holonomic 설정이 필요합니다.
일반적인 differential drive 설정과 다릅니다.

메카넘 휠 특성:
- 전후좌우 + 회전 동시 가능 (6-DOF planar motion)
- cmd_vel: linear.x, linear.y, angular.z 모두 사용
- 로컬 플래너: dwb_controller (DWB = Dynamic Window Approach)

[Task]
Set up Nav2 for JetRover with mecanum wheels and RPLIDAR A1.

Requirements:

1. [Block A] Nav2 패키지 설치:

   ```bash
   sudo apt update
   sudo apt install -y \
     ros-humble-nav2-bringup \
     ros-humble-navigation2 \
     ros-humble-nav2-dwb-controller \
     ros-humble-dwb-plugins \
     ros-humble-nav2-costmap-2d \
     ros-humble-nav2-map-server \
     ros-humble-nav2-lifecycle-manager \
     ros-humble-nav2-waypoint-follower

   # AMCL (localization)
   sudo apt install -y ros-humble-nav2-amcl

   # BehaviorTree Navigator
   sudo apt install -y ros-humble-nav2-bt-navigator
   ```

2. [Block B] 프로젝트 구조 생성:

   ```bash
   cd /home/ubuntu/AI_secretary_robot/src
   mkdir -p navigation/nav2_config/config
   mkdir -p navigation/nav2_config/launch
   mkdir -p navigation/nav2_config/maps
   mkdir -p navigation/nav2_config/params
   ```

3. [Block C] Nav2 파라미터 파일 작성:

   a. **nav2_params.yaml** (/src/navigation/nav2_config/params/nav2_params.yaml):

   ```yaml
   # Nav2 Parameters for JetRover (Mecanum Wheels)
   # Reference: plan.md section 4.3 (Navigation)

   bt_navigator:
     ros__parameters:
       use_sim_time: False
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       plugin_lib_names:
         - nav2_compute_path_to_pose_action_bt_node
         - nav2_follow_path_action_bt_node
         - nav2_back_up_action_bt_node
         - nav2_spin_action_bt_node
         - nav2_wait_action_bt_node
         - nav2_clear_costmap_service_bt_node
         - nav2_is_stuck_condition_bt_node
         - nav2_goal_reached_condition_bt_node
         - nav2_goal_updated_condition_bt_node
         - nav2_initial_pose_received_condition_bt_node
         - nav2_reinitialize_global_localization_service_bt_node
         - nav2_rate_controller_bt_node
         - nav2_distance_controller_bt_node
         - nav2_speed_controller_bt_node
         - nav2_truncate_path_action_bt_node
         - nav2_goal_updater_node_bt_node
         - nav2_recovery_node_bt_node
         - nav2_pipeline_sequence_bt_node
         - nav2_round_robin_node_bt_node
         - nav2_transform_available_condition_bt_node
         - nav2_time_expired_condition_bt_node
         - nav2_distance_traveled_condition_bt_node

   controller_server:
     ros__parameters:
       use_sim_time: False
       controller_frequency: 20.0  # 50ms cycle (plan.md requirement)
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.001  # Mecanum: enable y-axis
       min_theta_velocity_threshold: 0.001
       failure_tolerance: 0.3
       progress_checker_plugin: "progress_checker"
       goal_checker_plugins: ["general_goal_checker"]
       controller_plugins: ["FollowPath"]

       # Progress checker
       progress_checker:
         plugin: "nav2_controller::SimpleProgressChecker"
         required_movement_radius: 0.5
         movement_time_allowance: 10.0

       # Goal checker
       general_goal_checker:
         stateful: True
         plugin: "nav2_controller::SimpleGoalChecker"
         xy_goal_tolerance: 0.15
         yaw_goal_tolerance: 0.15

       # DWB Controller for mecanum wheels
       FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"

         # Mecanum-specific: enable holonomic motion
         debug_trajectory_details: False
         min_vel_x: -0.5  # Reverse allowed
         max_vel_x: 0.5   # plan.md: preferred_speed 0.5 m/s
         min_vel_y: -0.5  # Strafe left
         max_vel_y: 0.5   # Strafe right
         max_vel_theta: 1.0
         min_speed_xy: 0.0
         max_speed_xy: 0.5
         min_speed_theta: 0.0

         # Acceleration limits
         acc_lim_x: 2.5
         acc_lim_y: 2.5  # Mecanum: y-axis acceleration
         acc_lim_theta: 3.2
         decel_lim_x: -2.5
         decel_lim_y: -2.5
         decel_lim_theta: -3.2

         # Trajectory generation
         vx_samples: 20
         vy_samples: 20  # Mecanum: y-axis samples
         vtheta_samples: 40
         sim_time: 1.7
         linear_granularity: 0.05
         angular_granularity: 0.025
         transform_tolerance: 0.2
         xy_goal_tolerance: 0.15
         trans_stopped_velocity: 0.1
         short_circuit_trajectory_evaluation: True
         stateful: True

         # Critics (cost functions)
         critics:
           - "RotateToGoal"
           - "Oscillation"
           - "BaseObstacle"
           - "GoalAlign"
           - "PathAlign"
           - "PathDist"
           - "GoalDist"

         BaseObstacle.scale: 0.02
         PathAlign.scale: 32.0
         PathAlign.forward_point_distance: 0.1
         GoalAlign.scale: 24.0
         GoalAlign.forward_point_distance: 0.1
         PathDist.scale: 32.0
         GoalDist.scale: 24.0
         RotateToGoal.scale: 32.0
         RotateToGoal.slowing_factor: 5.0
         RotateToGoal.lookahead_time: -1.0

   local_costmap:
     local_costmap:
       ros__parameters:
         update_frequency: 5.0
         publish_frequency: 2.0
         global_frame: odom
         robot_base_frame: base_link
         use_sim_time: False
         rolling_window: true
         width: 3
         height: 3
         resolution: 0.05
         robot_radius: 0.20  # JetRover radius ~20cm
         plugins: ["voxel_layer", "inflation_layer"]

         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55

         voxel_layer:
           plugin: "nav2_costmap_2d::VoxelLayer"
           enabled: True
           publish_voxel_map: True
           origin_z: 0.0
           z_resolution: 0.05
           z_voxels: 16
           max_obstacle_height: 2.0
           mark_threshold: 0
           observation_sources: scan

           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 3.0
             raytrace_min_range: 0.0
             obstacle_max_range: 2.5
             obstacle_min_range: 0.0

         static_layer:
           map_subscribe_transient_local: True

         always_send_full_costmap: True

   global_costmap:
     global_costmap:
       ros__parameters:
         update_frequency: 1.0
         publish_frequency: 1.0
         global_frame: map
         robot_base_frame: base_link
         use_sim_time: False
         robot_radius: 0.20
         resolution: 0.05
         track_unknown_space: true
         plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

         obstacle_layer:
           plugin: "nav2_costmap_2d::ObstacleLayer"
           enabled: True
           observation_sources: scan

           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 3.0
             raytrace_min_range: 0.0
             obstacle_max_range: 2.5
             obstacle_min_range: 0.0

         static_layer:
           plugin: "nav2_costmap_2d::StaticLayer"
           map_subscribe_transient_local: True

         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55

         always_send_full_costmap: True

   planner_server:
     ros__parameters:
       expected_planner_frequency: 20.0
       use_sim_time: False
       planner_plugins: ["GridBased"]

       GridBased:
         plugin: "nav2_navfn_planner/NavfnPlanner"
         tolerance: 0.5
         use_astar: false
         allow_unknown: true

   behavior_server:
     ros__parameters:
       costmap_topic: local_costmap/costmap_raw
       footprint_topic: local_costmap/published_footprint
       cycle_frequency: 10.0
       behavior_plugins: ["spin", "backup", "wait"]

       spin:
         plugin: "nav2_behaviors::Spin"
       backup:
         plugin: "nav2_behaviors::BackUp"
       wait:
         plugin: "nav2_behaviors::Wait"

       global_frame: odom
       robot_base_frame: base_link
       transform_tolerance: 0.1
       use_sim_time: false
       simulate_ahead_time: 2.0
       max_rotational_vel: 1.0
       min_rotational_vel: 0.4
       rotational_acc_lim: 3.2

   waypoint_follower:
     ros__parameters:
       loop_rate: 20
       stop_on_failure: false
       waypoint_task_executor_plugin: "wait_at_waypoint"

       wait_at_waypoint:
         plugin: "nav2_waypoint_follower::WaitAtWaypoint"
         enabled: True
         waypoint_pause_duration: 200

   lifecycle_manager_navigation:
     ros__parameters:
       autostart: true
       node_names:
         - 'controller_server'
         - 'planner_server'
         - 'behavior_server'
         - 'bt_navigator'
         - 'waypoint_follower'
   ```

4. [Block D] Nav2 Launch 파일 작성:

   **nav2_bringup.launch.py** (/src/navigation/nav2_config/launch/nav2_bringup.launch.py):

   ```python
   #!/usr/bin/env python3
   """
   Nav2 Bringup for JetRover
   Launches Nav2 stack with mecanum wheel configuration
   """

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node

   def generate_launch_description():
       # Get package directories
       nav2_bringup_dir = get_package_share_directory('nav2_bringup')

       # Paths
       params_file = '/home/ubuntu/AI_secretary_robot/src/navigation/nav2_config/params/nav2_params.yaml'
       map_yaml_file = '/home/ubuntu/AI_secretary_robot/src/navigation/nav2_config/maps/default_map.yaml'

       # Launch arguments
       use_sim_time = LaunchConfiguration('use_sim_time', default='false')
       autostart = LaunchConfiguration('autostart', default='true')

       # Declare launch arguments
       declare_use_sim_time_cmd = DeclareLaunchArgument(
           'use_sim_time',
           default_value='false',
           description='Use simulation (Gazebo) clock if true'
       )

       declare_autostart_cmd = DeclareLaunchArgument(
           'autostart',
           default_value='true',
           description='Automatically startup the nav2 stack'
       )

       declare_params_file_cmd = DeclareLaunchArgument(
           'params_file',
           default_value=params_file,
           description='Full path to the ROS2 parameters file to use'
       )

       declare_map_yaml_cmd = DeclareLaunchArgument(
           'map',
           default_value=map_yaml_file,
           description='Full path to map yaml file to load'
       )

       # Nav2 Bringup
       nav2_bringup_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
           ),
           launch_arguments={
               'use_sim_time': use_sim_time,
               'autostart': autostart,
               'params_file': params_file,
               'map': map_yaml_file,
           }.items()
       )

       ld = LaunchDescription()

       # Add declarations
       ld.add_action(declare_use_sim_time_cmd)
       ld.add_action(declare_autostart_cmd)
       ld.add_action(declare_params_file_cmd)
       ld.add_action(declare_map_yaml_cmd)

       # Add Nav2
       ld.add_action(nav2_bringup_launch)

       return ld
   ```

5. [Block E] package.xml 생성:

   **/src/navigation/nav2_config/package.xml**:

   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>nav2_config</name>
     <version>1.0.0</version>
     <description>Nav2 configuration for JetRover mecanum wheels</description>
     <maintainer email="rover@jetrover.com">JetRover Team</maintainer>
     <license>Apache-2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <exec_depend>nav2_bringup</exec_depend>
     <exec_depend>nav2_dwb_controller</exec_depend>
     <exec_depend>dwb_plugins</exec_depend>
     <exec_depend>nav2_lifecycle_manager</exec_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

6. [Block F] CMakeLists.txt 생성:

   **/src/navigation/nav2_config/CMakeLists.txt**:

   ```cmake
   cmake_minimum_required(VERSION 3.5)
   project(nav2_config)

   find_package(ament_cmake REQUIRED)

   # Install launch files
   install(DIRECTORY
     launch
     params
     maps
     DESTINATION share/${PROJECT_NAME}/
   )

   ament_package()
   ```

7. [Block G] 초기 빈 지도 생성:

   **/src/navigation/nav2_config/maps/default_map.yaml**:
   ```yaml
   image: default_map.pgm
   resolution: 0.05
   origin: [-10.0, -10.0, 0.0]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.196
   ```

   **/src/navigation/nav2_config/maps/default_map.pgm**:
   빈 400x400 픽셀 회색 이미지 (SLAM으로 나중에 업데이트)

8. [Block H] 빌드 및 검증:

   ```bash
   cd /home/ubuntu/AI_secretary_robot
   source /opt/ros/humble/setup.bash
   colcon build --packages-select nav2_config
   source install/setup.bash

   # Nav2 실행 테스트
   ros2 launch nav2_config nav2_bringup.launch.py
   ```

[Constraints]
- Mecanum wheel holonomic 설정 필수 (vy_samples > 0)
- LiDAR topic: /scan (360도 커버리지)
- Odom topic: /odom (ros_robot_controller_cpp에서 발행 필요)
- Transform: map → odom → base_link (SLAM에서 제공)

[Verification]
ros2 topic list | grep nav2
ros2 node list | grep controller_server
ros2 param get /controller_server FollowPath.max_vel_y  # Should be 0.5

[Output]
- /src/navigation/nav2_config/ 패키지 생성
- nav2_params.yaml (메카넘 휠 특화 설정)
- nav2_bringup.launch.py
- 빌드 성공 및 노드 실행 확인
```

---

## 🗺️ Prompt H2: SLAM Toolbox 설치 및 설정

```
[Context]
- Nav2가 설치되어 작동 중
- 하드웨어: RPLIDAR A1 (360도 LiDAR, /scan 토픽)
- 목표: slam_toolbox를 사용하여 실시간 지도 작성 및 localization
- Output: SLAM 설정 파일 및 launch 파일

[Task]
Set up SLAM Toolbox for real-time mapping and localization.

Requirements:

1. [Block A] SLAM Toolbox 설치:

   ```bash
   sudo apt install -y ros-humble-slam-toolbox
   ```

2. [Block B] 디렉토리 구조:

   ```bash
   cd /home/ubuntu/AI_secretary_robot/src/navigation
   mkdir -p slam_config/config
   mkdir -p slam_config/launch
   mkdir -p slam_config/maps
   ```

3. [Block C] SLAM 파라미터 파일:

   **slam_params.yaml** (/src/navigation/slam_config/config/slam_params.yaml):

   ```yaml
   # SLAM Toolbox Parameters for JetRover
   # Reference: plan.md section 4.3 (SLAM)
   # Target: 위치오차 ±5cm, 50ms latency

   slam_toolbox:
     ros__parameters:
       # ROS Parameters
       odom_frame: odom
       map_frame: map
       base_frame: base_link
       scan_topic: /scan
       use_map_saver: true
       mode: mapping  # Options: mapping, localization

       # Solver Parameters
       solver_plugin: solver_plugins::CeresSolver
       ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
       ceres_preconditioner: SCHUR_JACOBI
       ceres_trust_strategy: LEVENBERG_MARQUARDT
       ceres_dogleg_type: TRADITIONAL_DOGLEG
       ceres_loss_function: None

       # ROS Parameters
       transform_publish_period: 0.02  # 50Hz = 20ms (plan.md: 50ms target)
       map_update_interval: 5.0
       resolution: 0.05  # 5cm resolution (plan.md: ±5cm accuracy)
       max_laser_range: 12.0  # RPLIDAR A1 max range
       minimum_time_interval: 0.5
       transform_timeout: 0.2
       tf_buffer_duration: 30.0
       stack_size_to_use: 40000000
       enable_interactive_mode: true

       # General Parameters
       use_scan_matching: true
       use_scan_barycenter: true
       minimum_travel_distance: 0.3  # m
       minimum_travel_heading: 0.5   # rad
       scan_buffer_size: 10
       scan_buffer_maximum_scan_distance: 10.0
       link_match_minimum_response_fine: 0.1
       link_scan_maximum_distance: 1.5
       loop_search_maximum_distance: 3.0
       do_loop_closing: true
       loop_match_minimum_chain_size: 10
       loop_match_maximum_variance_coarse: 3.0
       loop_match_minimum_response_coarse: 0.35
       loop_match_minimum_response_fine: 0.45

       # Correlation Parameters (RPLIDAR A1 specific)
       correlation_search_space_dimension: 0.5
       correlation_search_space_resolution: 0.01
       correlation_search_space_smear_deviation: 0.1

       # Scan Matcher Parameters
       coarse_search_angle_offset: 0.349  # 20 degrees
       coarse_angle_resolution: 0.0349    # 2 degrees
       minimum_angle_penalty: 0.9
       minimum_distance_penalty: 0.5
       use_response_expansion: true

       # Loop Closure Parameters
       loop_search_space_dimension: 8.0
       loop_search_space_resolution: 0.05
       loop_search_space_smear_deviation: 0.03

       # Distance Penalty Parameters
       distance_variance_penalty: 0.5
       angle_variance_penalty: 1.0
   ```

4. [Block D] Mapping Launch 파일:

   **online_async_launch.py** (/src/navigation/slam_config/launch/online_async_launch.py):

   ```python
   #!/usr/bin/env python3
   """
   SLAM Toolbox Online Async Mapping
   Real-time SLAM with map updates
   """

   import os
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory

   def generate_launch_description():
       # Paths
       config_file = '/home/ubuntu/AI_secretary_robot/src/navigation/slam_config/config/slam_params.yaml'

       # Launch configuration
       use_sim_time = LaunchConfiguration('use_sim_time')

       declare_use_sim_time_argument = DeclareLaunchArgument(
           'use_sim_time',
           default_value='false',
           description='Use simulation/Gazebo clock'
       )

       # SLAM Toolbox Node
       slam_toolbox_node = Node(
           parameters=[
               config_file,
               {'use_sim_time': use_sim_time}
           ],
           package='slam_toolbox',
           executable='async_slam_toolbox_node',
           name='slam_toolbox',
           output='screen',
           remappings=[
               ('/scan', '/scan'),
           ]
       )

       ld = LaunchDescription()
       ld.add_action(declare_use_sim_time_argument)
       ld.add_action(slam_toolbox_node)

       return ld
   ```

5. [Block E] Localization Launch 파일:

   **localization_launch.py** (/src/navigation/slam_config/launch/localization_launch.py):

   ```python
   #!/usr/bin/env python3
   """
   SLAM Toolbox Localization Mode
   Use pre-built map for localization only
   """

   import os
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node

   def generate_launch_description():
       # Paths
       config_file = '/home/ubuntu/AI_secretary_robot/src/navigation/slam_config/config/slam_params.yaml'
       map_file = '/home/ubuntu/AI_secretary_robot/src/navigation/slam_config/maps/saved_map.posegraph'

       use_sim_time = LaunchConfiguration('use_sim_time')

       declare_use_sim_time_argument = DeclareLaunchArgument(
           'use_sim_time',
           default_value='false',
           description='Use simulation/Gazebo clock'
       )

       declare_map_file_argument = DeclareLaunchArgument(
           'map_file',
           default_value=map_file,
           description='Full path to saved map file'
       )

       # Localization mode
       localization_node = Node(
           parameters=[
               config_file,
               {'use_sim_time': use_sim_time},
               {'mode': 'localization'},
               {'map_file_name': LaunchConfiguration('map_file')}
           ],
           package='slam_toolbox',
           executable='localization_slam_toolbox_node',
           name='slam_toolbox',
           output='screen'
       )

       ld = LaunchDescription()
       ld.add_action(declare_use_sim_time_argument)
       ld.add_action(declare_map_file_argument)
       ld.add_action(localization_node)

       return ld
   ```

6. [Block F] 지도 저장 스크립트:

   **save_map.sh** (/src/navigation/slam_config/scripts/save_map.sh):

   ```bash
   #!/bin/bash
   # Save current SLAM map to file

   MAP_DIR="/home/ubuntu/AI_secretary_robot/src/navigation/slam_config/maps"
   TIMESTAMP=$(date +%Y%m%d_%H%M%S)
   MAP_NAME="${MAP_DIR}/map_${TIMESTAMP}"

   echo "Saving map to: ${MAP_NAME}"

   # Call SLAM Toolbox service to serialize map
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "{filename: '${MAP_NAME}'}"

   if [ $? -eq 0 ]; then
       echo "Map saved successfully!"
       echo "Files created:"
       ls -lh ${MAP_NAME}.*

       # Create symlink to latest map
       ln -sf ${MAP_NAME}.posegraph ${MAP_DIR}/saved_map.posegraph
       ln -sf ${MAP_NAME}.data ${MAP_DIR}/saved_map.data

       echo "Symlink created: saved_map.posegraph -> map_${TIMESTAMP}.posegraph"
   else
       echo "Failed to save map!"
       exit 1
   fi
   ```

   ```bash
   chmod +x /home/ubuntu/AI_secretary_robot/src/navigation/slam_config/scripts/save_map.sh
   ```

7. [Block G] Package.xml & CMakeLists.txt:

   **/src/navigation/slam_config/package.xml**:
   ```xml
   <?xml version="1.0"?>
   <package format="3">
     <name>slam_config</name>
     <version>1.0.0</version>
     <description>SLAM Toolbox configuration for JetRover</description>
     <maintainer email="rover@jetrover.com">JetRover Team</maintainer>
     <license>Apache-2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>
     <exec_depend>slam_toolbox</exec_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

   **/src/navigation/slam_config/CMakeLists.txt**:
   ```cmake
   cmake_minimum_required(VERSION 3.5)
   project(slam_config)

   find_package(ament_cmake REQUIRED)

   install(DIRECTORY
     launch
     config
     scripts
     maps
     DESTINATION share/${PROJECT_NAME}/
   )

   install(PROGRAMS
     scripts/save_map.sh
     DESTINATION lib/${PROJECT_NAME}
   )

   ament_package()
   ```

8. [Block H] 빌드 및 검증:

   ```bash
   cd /home/ubuntu/AI_secretary_robot
   colcon build --packages-select slam_config
   source install/setup.bash

   # SLAM 실행 (매핑 모드)
   ros2 launch slam_config online_async_launch.py

   # 지도 저장 (별도 터미널)
   ros2 run slam_config save_map.sh

   # Localization 모드로 전환
   ros2 launch slam_config localization_launch.py
   ```

[Constraints]
- LiDAR 토픽: /scan (RPLIDAR A1에서 발행)
- Odom 토픽: /odom (정확한 odometry 필수)
- TF: odom → base_link (ros_robot_controller_cpp에서 발행)
- 지도 해상도: 5cm (plan.md: ±5cm accuracy)

[Verification]
ros2 topic echo /map --once  # Check map is published
ros2 run tf2_ros tf2_echo map base_link  # Check transform
ros2 service list | grep slam_toolbox  # Check services available

[Output]
- /src/navigation/slam_config/ 패키지
- slam_params.yaml
- online_async_launch.py (매핑)
- localization_launch.py (로컬라이제이션)
- save_map.sh (지도 저장 스크립트)
```

---

## 🚀 Prompt H3: 통합 Bringup 시스템

```
[Context]
- Nav2와 SLAM이 개별적으로 작동함
- 하드웨어 드라이버들이 src/hardware/와 src/control/에 분산되어 있음
- 목표: 전체 시스템을 한 번에 시작하는 통합 launch 시스템 구축
- Output: host_bringup 패키지 생성

[Reference]
plan.md section 6.4 시스템 실행 순서:
1. Jetson 전원 ON (nvpmodel)
2. 팬 제어
3. ZRAM 설정
4. **Host 시작** ← 이 단계 자동화
5. Brain 시작 (Docker)
6. Health Check
7. YOLO 워밍업

[Task]
Create unified bringup system for JetRover Host.

Requirements:

1. [Block A] 디렉토리 구조:

   ```bash
   cd /home/ubuntu/AI_secretary_robot/src
   mkdir -p bringup/host_bringup/launch
   mkdir -p bringup/host_bringup/config
   mkdir -p bringup/host_bringup/scripts
   ```

2. [Block B] 메인 Launch 파일:

   **jetrover_host_main.launch.py** (/src/bringup/host_bringup/launch/jetrover_host_main.launch.py):

   ```python
   #!/usr/bin/env python3
   """
   JetRover Host Main Launch File
   Brings up all hardware drivers, SLAM, and Nav2

   Launch order:
   1. Hardware drivers (sensors, motors)
   2. SLAM Toolbox (mapping/localization)
   3. Nav2 (navigation stack)
   4. MoveIt2 (arm control) - optional
   """

   import os
   from launch import LaunchDescription
   from launch.actions import (
       DeclareLaunchArgument,
       IncludeLaunchDescription,
       GroupAction,
       TimerAction
   )
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch_ros.actions import Node, SetParameter
   from launch_ros.substitutions import FindPackageShare

   def generate_launch_description():
       # Launch arguments
       use_slam = LaunchConfiguration('use_slam', default='true')
       use_nav2 = LaunchConfiguration('use_nav2', default='true')
       use_arm = LaunchConfiguration('use_arm', default='false')
       slam_mode = LaunchConfiguration('slam_mode', default='mapping')  # mapping or localization

       # Declare arguments
       declare_use_slam = DeclareLaunchArgument(
           'use_slam',
           default_value='true',
           description='Launch SLAM Toolbox'
       )

       declare_use_nav2 = DeclareLaunchArgument(
           'use_nav2',
           default_value='true',
           description='Launch Nav2 navigation stack'
       )

       declare_use_arm = DeclareLaunchArgument(
           'use_arm',
           default_value='false',
           description='Launch MoveIt2 for robot arm'
       )

       declare_slam_mode = DeclareLaunchArgument(
           'slam_mode',
           default_value='mapping',
           description='SLAM mode: mapping or localization'
       )

       # ========================================
       # Phase 1: Hardware Drivers (0s delay)
       # ========================================

       # LiDAR
       lidar_node = Node(
           package='lidar_cpp',
           executable='lidar_node',
           name='lidar_node',
           output='screen',
           parameters=[{
               'serial_port': '/dev/ttyUSB0',
               'frame_id': 'laser_frame',
               'scan_frequency': 10.0
           }]
       )

       # IMU
       imu_node = Node(
           package='imu_bridge_cpp',
           executable='imu_bridge_node',
           name='imu_bridge_node',
           output='screen'
       )

       # Camera (Orbbec Dabai DCW)
       camera_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('orbbec_camera'),
                   'launch',
                   'dabai_dcw.launch.py'
               ])
           ])
       )

       # Battery Monitor
       battery_node = Node(
           package='battery_cpp',
           executable='battery_node',
           name='battery_node',
           output='screen'
       )

       # Motor Controller
       motor_controller_node = Node(
           package='ros_robot_controller_cpp',
           executable='ros_robot_controller_cpp_node',
           name='motor_controller',
           output='screen',
           parameters=[{
               'serial_port': '/dev/ttyTHS0',
               'baud_rate': 115200,
               'publish_odom': True,
               'odom_frame': 'odom',
               'base_frame': 'base_link'
           }]
       )

       # Robot State Publisher (TF tree)
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           output='screen',
           parameters=[{
               'robot_description': open('/home/ubuntu/AI_secretary_robot/src/control/jetrover_arm_moveit/urdf/jetrover.xacro').read()
           }]
       )

       # ========================================
       # Phase 2: SLAM (2s delay)
       # ========================================

       slam_launch = TimerAction(
           period=2.0,
           actions=[
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource([
                       '/home/ubuntu/AI_secretary_robot/src/navigation/slam_config/launch/',
                       slam_mode,
                       '_launch.py'
                   ])
               )
           ]
       )

       # ========================================
       # Phase 3: Nav2 (5s delay)
       # ========================================

       nav2_launch = TimerAction(
           period=5.0,
           actions=[
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource([
                       '/home/ubuntu/AI_secretary_robot/src/navigation/nav2_config/launch/nav2_bringup.launch.py'
                   ])
               )
           ]
       )

       # ========================================
       # Phase 4: MoveIt2 (optional, 3s delay)
       # ========================================

       moveit_launch = TimerAction(
           period=3.0,
           actions=[
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource([
                       PathJoinSubstitution([
                           FindPackageShare('jetrover_arm_moveit'),
                           'launch',
                           'moveit_demo.launch.py'
                       ])
                   ])
               )
           ]
       )

       # Build launch description
       ld = LaunchDescription()

       # Add arguments
       ld.add_action(declare_use_slam)
       ld.add_action(declare_use_nav2)
       ld.add_action(declare_use_arm)
       ld.add_action(declare_slam_mode)

       # Add hardware drivers (immediate)
       ld.add_action(lidar_node)
       ld.add_action(imu_node)
       ld.add_action(camera_launch)
       ld.add_action(battery_node)
       ld.add_action(motor_controller_node)
       ld.add_action(robot_state_publisher)

       # Add SLAM (delayed)
       ld.add_action(slam_launch)

       # Add Nav2 (delayed)
       ld.add_action(nav2_launch)

       # Add MoveIt2 (optional, delayed)
       # ld.add_action(moveit_launch)  # Uncomment to enable

       return ld
   ```

3. [Block C] 간단한 시작 스크립트:

   **start_host.sh** (/src/bringup/host_bringup/scripts/start_host.sh):

   ```bash
   #!/bin/bash
   # JetRover Host System Startup Script
   # Usage: ./start_host.sh [mapping|localization]

   set -e

   SLAM_MODE="${1:-mapping}"

   echo "========================================="
   echo "JetRover Host System Starting"
   echo "SLAM Mode: $SLAM_MODE"
   echo "========================================="

   # Source ROS2
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/AI_secretary_robot/install/setup.bash

   # Check hardware connections
   echo "Checking hardware..."

   # LiDAR (USB)
   if [ ! -e /dev/ttyUSB0 ]; then
       echo "WARNING: LiDAR not found at /dev/ttyUSB0"
   else
       echo "✓ LiDAR detected"
   fi

   # Motor controller (UART)
   if [ ! -e /dev/ttyTHS0 ]; then
       echo "WARNING: Motor controller not found at /dev/ttyTHS0"
   else
       echo "✓ Motor controller detected"
   fi

   # Launch main system
   echo ""
   echo "Launching Host system..."
   ros2 launch host_bringup jetrover_host_main.launch.py \
       slam_mode:=$SLAM_MODE \
       use_slam:=true \
       use_nav2:=true
   ```

   ```bash
   chmod +x /home/ubuntu/AI_secretary_robot/src/bringup/host_bringup/scripts/start_host.sh
   ```

4. [Block D] Package files:

   **package.xml**:
   ```xml
   <?xml version="1.0"?>
   <package format="3">
     <name>host_bringup</name>
     <version>1.0.0</version>
     <description>Unified bringup system for JetRover Host</description>
     <maintainer email="rover@jetrover.com">JetRover Team</maintainer>
     <license>Apache-2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <exec_depend>lidar_cpp</exec_depend>
     <exec_depend>imu_bridge_cpp</exec_depend>
     <exec_depend>orbbec_camera</exec_depend>
     <exec_depend>battery_cpp</exec_depend>
     <exec_depend>ros_robot_controller_cpp</exec_depend>
     <exec_depend>robot_state_publisher</exec_depend>
     <exec_depend>slam_config</exec_depend>
     <exec_depend>nav2_config</exec_depend>
     <exec_depend>jetrover_arm_moveit</exec_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

   **CMakeLists.txt**:
   ```cmake
   cmake_minimum_required(VERSION 3.5)
   project(host_bringup)

   find_package(ament_cmake REQUIRED)

   install(DIRECTORY
     launch
     config
     scripts
     DESTINATION share/${PROJECT_NAME}/
   )

   install(PROGRAMS
     scripts/start_host.sh
     DESTINATION lib/${PROJECT_NAME}
   )

   ament_package()
   ```

5. [Block E] 빌드 및 실행:

   ```bash
   cd /home/ubuntu/AI_secretary_robot
   colcon build --packages-select host_bringup
   source install/setup.bash

   # 매핑 모드로 시작
   ros2 run host_bringup start_host.sh mapping

   # 또는 직접 launch
   ros2 launch host_bringup jetrover_host_main.launch.py
   ```

[Constraints]
- 노드 시작 순서 중요: 하드웨어 → SLAM → Nav2
- TF tree 완성 필수: map → odom → base_link
- /scan, /odom 토픽 발행 확인

[Verification]
# 모든 노드 실행 확인
ros2 node list

# TF tree 확인
ros2 run tf2_tools view_frames

# 토픽 확인
ros2 topic list

# Expected output includes:
# /scan, /odom, /map, /cmd_vel, /joint_states

[Output]
- /src/bringup/host_bringup/ 패키지
- jetrover_host_main.launch.py (통합 launch)
- start_host.sh (실행 스크립트)
- 전체 시스템 한 번에 시작 가능
```

---

## 🛑 Prompt H4: E-Stop (비상 정지) 시스템

```
[Context]
- 모터 컨트롤러(ros_robot_controller_cpp)가 작동 중
- 목표: 하드웨어 버튼 + 소프트웨어 API로 비상 정지 구현
- Output: E-Stop 노드 및 통합

[Safety Critical]
⚠️ E-Stop은 Safety-Critical 시스템입니다.
- 응답 시간: < 100ms
- 우선순위: ABSOLUTE (OOM score -1000)
- 모든 모터 즉시 정지
- 재시작 전까지 이동 금지

[Task]
Implement Emergency Stop system for JetRover.

Requirements:

1. [Block A] E-Stop 노드 C++ 구현:

   **emergency_stop_node.cpp** (/src/control/ros_robot_controller_cpp/src/emergency_stop_node.cpp):

   ```cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/bool.hpp>
   #include <std_srvs/srv/trigger.hpp>
   #include <geometry_msgs/msg/twist.hpp>

   #include <chrono>
   #include <memory>
   #include <atomic>

   using namespace std::chrono_literals;

   class EmergencyStopNode : public rclcpp::Node {
   public:
       EmergencyStopNode() : Node("emergency_stop_node"), estop_active_(false) {
           // Publisher: E-Stop status
           estop_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
               "/emergency_stop", rclcpp::QoS(10).reliable()
           );

           // Publisher: cmd_vel override (stop command)
           cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
               "/cmd_vel", 10
           );

           // Service: Activate E-Stop
           activate_service_ = this->create_service<std_srvs::srv::Trigger>(
               "/emergency_stop/activate",
               std::bind(&EmergencyStopNode::activate_callback, this,
                         std::placeholders::_1, std::placeholders::_2)
           );

           // Service: Deactivate E-Stop (reset)
           deactivate_service_ = this->create_service<std_srvs::srv::Trigger>(
               "/emergency_stop/deactivate",
               std::bind(&EmergencyStopNode::deactivate_callback, this,
                         std::placeholders::_1, std::placeholders::_2)
           );

           // TODO: GPIO input for hardware button (requires libgpiod)
           // For now, use software API only

           // Timer: Publish E-Stop status at 10Hz
           status_timer_ = this->create_wall_timer(
               100ms, std::bind(&EmergencyStopNode::publish_status, this)
           );

           RCLCPP_INFO(this->get_logger(), "Emergency Stop Node started");
           RCLCPP_INFO(this->get_logger(), "Services:");
           RCLCPP_INFO(this->get_logger(), "  - /emergency_stop/activate");
           RCLCPP_INFO(this->get_logger(), "  - /emergency_stop/deactivate");
       }

   private:
       void activate_callback(
           const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response
       ) {
           estop_active_.store(true);

           // Send stop command immediately
           auto stop_cmd = geometry_msgs::msg::Twist();
           stop_cmd.linear.x = 0.0;
           stop_cmd.linear.y = 0.0;
           stop_cmd.angular.z = 0.0;
           cmd_vel_pub_->publish(stop_cmd);

           RCLCPP_ERROR(this->get_logger(), "🛑 EMERGENCY STOP ACTIVATED");

           response->success = true;
           response->message = "Emergency stop activated. All motion stopped.";
       }

       void deactivate_callback(
           const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response
       ) {
           if (!estop_active_.load()) {
               response->success = false;
               response->message = "Emergency stop is not active";
               return;
           }

           estop_active_.store(false);
           RCLCPP_WARN(this->get_logger(), "✓ Emergency stop deactivated. System ready.");

           response->success = true;
           response->message = "Emergency stop deactivated. Motion enabled.";
       }

       void publish_status() {
           auto msg = std_msgs::msg::Bool();
           msg.data = estop_active_.load();
           estop_status_pub_->publish(msg);
       }

       std::atomic<bool> estop_active_;
       rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_status_pub_;
       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
       rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activate_service_;
       rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deactivate_service_;
       rclcpp::TimerBase::SharedPtr status_timer_;
   };

   int main(int argc, char** argv) {
       rclcpp::init(argc, argv);

       // Set high priority (requires sudo or capabilities)
       // sched_param param;
       // param.sched_priority = 99;
       // sched_setscheduler(0, SCHED_FIFO, &param);

       auto node = std::make_shared<EmergencyStopNode>();
       rclcpp::spin(node);
       rclcpp::shutdown();
       return 0;
   }
   ```

2. [Block B] Motor Controller에 E-Stop 핸들러 추가:

   ros_robot_controller_cpp_node.cpp에 추가:

   ```cpp
   // In class definition:
   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
   std::atomic<bool> estop_active_{false};

   // In constructor:
   estop_sub_ = create_subscription<std_msgs::msg::Bool>(
       "/emergency_stop", 10,
       [this](const std_msgs::msg::Bool::SharedPtr msg) {
           estop_active_.store(msg->data);
           if (msg->data) {
               // Stop all motors immediately
               stop_all_motors();
               RCLCPP_ERROR(get_logger(), "E-Stop: Motors halted");
           }
       }
   );

   // In cmd_vel callback (before processing commands):
   if (estop_active_.load()) {
       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "E-Stop active - ignoring cmd_vel");
       return;
   }
   ```

3. [Block C] CMakeLists.txt 업데이트:

   ros_robot_controller_cpp/CMakeLists.txt에 추가:

   ```cmake
   add_executable(emergency_stop_node src/emergency_stop_node.cpp)
   ament_target_dependencies(emergency_stop_node
     rclcpp
     std_msgs
     std_srvs
     geometry_msgs
   )
   install(TARGETS emergency_stop_node DESTINATION lib/${PROJECT_NAME})
   ```

4. [Block D] Launch 파일에 통합:

   jetrover_host_main.launch.py에 추가:

   ```python
   # E-Stop Node (highest priority)
   estop_node = Node(
       package='ros_robot_controller_cpp',
       executable='emergency_stop_node',
       name='emergency_stop',
       output='screen'
   )

   # Add to launch description (first!)
   ld.add_action(estop_node)
   ```

5. [Block E] 테스트 스크립트:

   **test_estop.sh**:
   ```bash
   #!/bin/bash
   echo "Testing E-Stop..."

   # Activate
   echo "1. Activating E-Stop..."
   ros2 service call /emergency_stop/activate std_srvs/srv/Trigger

   # Check status
   echo "2. Checking status..."
   ros2 topic echo /emergency_stop --once

   # Try to send cmd_vel (should be ignored)
   echo "3. Sending cmd_vel (should be ignored)..."
   ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

   # Deactivate
   echo "4. Deactivating E-Stop..."
   ros2 service call /emergency_stop/deactivate std_srvs/srv/Trigger

   echo "Test complete!"
   ```

[Hardware Integration - Optional]
하드웨어 버튼 연결 (GPIO):
```cpp
#include <gpiod.h>

// GPIO 핀 번호 (Jetson Orin Nano)
#define ESTOP_GPIO_CHIP "gpiochip0"
#define ESTOP_GPIO_LINE 12

// Initialize GPIO
gpiod_chip* chip = gpiod_chip_open_by_name(ESTOP_GPIO_CHIP);
gpiod_line* line = gpiod_chip_get_line(chip, ESTOP_GPIO_LINE);
gpiod_line_request_input(line, "estop_button");

// Poll GPIO (in timer callback)
int val = gpiod_line_get_value(line);
if (val == 0) {  // Button pressed (active low)
    activate_estop();
}
```

[Constraints]
- 응답 시간: < 100ms
- 우선순위: Real-time scheduling (SCHED_FIFO)
- OOM score: -1000 (절대 kill 금지)

[Verification]
ros2 service call /emergency_stop/activate std_srvs/srv/Trigger
ros2 topic echo /emergency_stop
ros2 topic pub /cmd_vel ...  # Should be ignored when active

[Output]
- emergency_stop_node 실행 파일
- /emergency_stop 토픽 (Bool)
- /emergency_stop/activate 서비스
- /emergency_stop/deactivate 서비스
- Motor controller에 E-Stop 핸들러 통합
```

---

## 🔗 Prompt H5: TF Tree 완성

```
[Context]
- 현재 일부 TF만 발행됨
- Nav2와 SLAM이 제대로 작동하려면 완전한 TF tree 필요
- 목표: map → odom → base_link → 모든 센서/링크 연결

[Task]
Complete the TF tree for JetRover.

Requirements:

1. [Block A] Static TF Publisher:

   **static_transforms.launch.py** (/src/bringup/host_bringup/launch/static_transforms.launch.py):

   ```python
   #!/usr/bin/env python3
   """
   Static Transform Publishers for JetRover
   Publishes fixed transforms between frames
   """

   from launch import LaunchDescription
   from launch_ros.actions import Node
   import math

   def generate_launch_description():
       # All static transforms
       static_transforms = [
           # base_link → laser_frame (LiDAR)
           {
               'parent': 'base_link',
               'child': 'laser_frame',
               'xyz': [0.0, 0.0, 0.15],  # 15cm above base
               'rpy': [0.0, 0.0, 0.0]
           },

           # base_link → camera_link (Orbbec Dabai DCW)
           {
               'parent': 'base_link',
               'child': 'camera_link',
               'xyz': [0.10, 0.0, 0.20],  # 10cm forward, 20cm up
               'rpy': [0.0, 0.0, 0.0]
           },

           # camera_link → camera_color_frame
           {
               'parent': 'camera_link',
               'child': 'camera_color_frame',
               'xyz': [0.0, 0.0, 0.0],
               'rpy': [0.0, 0.0, 0.0]
           },

           # camera_link → camera_depth_frame
           {
               'parent': 'camera_link',
               'child': 'camera_depth_frame',
               'xyz': [0.0, 0.0, 0.0],
               'rpy': [0.0, 0.0, 0.0]
           },

           # base_link → imu_link (IMU)
           {
               'parent': 'base_link',
               'child': 'imu_link',
               'xyz': [0.0, 0.0, 0.05],  # 5cm above base
               'rpy': [0.0, 0.0, 0.0]
           },

           # base_link → arm_base_link (Robot Arm)
           {
               'parent': 'base_link',
               'child': 'arm_base_link',
               'xyz': [-0.05, 0.0, 0.10],  # 5cm back, 10cm up
               'rpy': [0.0, 0.0, 0.0]
           },
       ]

       # Create static transform publisher nodes
       nodes = []
       for tf in static_transforms:
           node = Node(
               package='tf2_ros',
               executable='static_transform_publisher',
               name=f"static_tf_{tf['child']}",
               arguments=[
                   str(tf['xyz'][0]),
                   str(tf['xyz'][1]),
                   str(tf['xyz'][2]),
                   str(tf['rpy'][0]),
                   str(tf['rpy'][1]),
                   str(tf['rpy'][2]),
                   tf['parent'],
                   tf['child']
               ]
           )
           nodes.append(node)

       ld = LaunchDescription()
       for node in nodes:
           ld.add_action(node)

       return ld
   ```

2. [Block B] Odom → Base_link Publisher:

   ros_robot_controller_cpp에 추가 (이미 부분적으로 구현되어 있을 수 있음):

   ```cpp
   #include <tf2_ros/transform_broadcaster.h>
   #include <geometry_msgs/msg/transform_stamped.hpp>

   // In class:
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

   // In constructor:
   tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

   // In odometry calculation callback (주기적 호출):
   void publish_odometry() {
       // ... odometry calculation ...

       // Publish odom → base_link transform
       geometry_msgs::msg::TransformStamped transform;
       transform.header.stamp = this->now();
       transform.header.frame_id = "odom";
       transform.child_frame_id = "base_link";

       transform.transform.translation.x = odom_x_;
       transform.transform.translation.y = odom_y_;
       transform.transform.translation.z = 0.0;

       tf2::Quaternion q;
       q.setRPY(0, 0, odom_theta_);
       transform.transform.rotation.x = q.x();
       transform.transform.rotation.y = q.y();
       transform.transform.rotation.z = q.z();
       transform.transform.rotation.w = q.w();

       tf_broadcaster_->sendTransform(transform);
   }
   ```

3. [Block C] TF 검증 스크립트:

   **check_tf_tree.sh** (/src/bringup/host_bringup/scripts/check_tf_tree.sh):

   ```bash
   #!/bin/bash
   echo "Checking TF tree..."

   # Generate PDF
   ros2 run tf2_tools view_frames

   # Check critical transforms
   echo ""
   echo "Critical transforms:"

   echo -n "map → odom: "
   ros2 run tf2_ros tf2_echo map odom 2>/dev/null && echo "✓" || echo "✗"

   echo -n "odom → base_link: "
   ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null && echo "✓" || echo "✗"

   echo -n "base_link → laser_frame: "
   ros2 run tf2_ros tf2_echo base_link laser_frame 2>/dev/null && echo "✓" || echo "✗"

   echo -n "base_link → camera_link: "
   ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null && echo "✓" || echo "✗"

   echo ""
   echo "TF tree PDF generated: frames_YYYY-MM-DD_HH.MM.SS.pdf"
   ls -lt frames_*.pdf | head -1
   ```

4. [Block D] Launch 파일에 통합:

   jetrover_host_main.launch.py에 추가:

   ```python
   # Static TF Publishers
   static_tf_launch = IncludeLaunchDescription(
       PythonLaunchDescriptionSource([
           '/home/ubuntu/AI_secretary_robot/src/bringup/host_bringup/launch/static_transforms.launch.py'
       ])
   )

   # Add early in launch description
   ld.add_action(static_tf_launch)
   ```

[Expected TF Tree]
```
map (SLAM)
 └─ odom (motor controller)
     └─ base_link (robot center)
         ├─ laser_frame (LiDAR)
         ├─ camera_link (RGB-D camera)
         │   ├─ camera_color_frame
         │   └─ camera_depth_frame
         ├─ imu_link (IMU)
         └─ arm_base_link (robot arm)
             └─ ... (MoveIt2 arm joints)
```

[Verification]
ros2 run tf2_tools view_frames
evince frames_*.pdf  # View TF tree diagram

[Output]
- static_transforms.launch.py
- odom → base_link transform in motor controller
- check_tf_tree.sh 스크립트
- 완전한 TF tree
```

---

## ⚙️ Prompt H6: systemd 서비스 & 자동화

```
[Context]
- 전체 시스템이 수동으로 작동함
- 목표: 부팅 시 자동 시작, systemd 서비스 관리
- Output: systemd 서비스 파일 및 설치 스크립트

[Task]
Create systemd service for automatic startup.

Requirements:

1. [Block A] systemd 서비스 파일:

   **jetrover-host.service** (/src/bringup/host_bringup/systemd/jetrover-host.service):

   ```ini
   [Unit]
   Description=JetRover Host System (ROS2 Navigation & Control)
   After=network-online.target
   Wants=network-online.target

   [Service]
   Type=simple
   User=ubuntu
   Group=ubuntu
   WorkingDirectory=/home/ubuntu/AI_secretary_robot

   # Environment
   Environment="ROS_DOMAIN_ID=42"
   Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"

   # Source ROS2 and workspace
   ExecStartPre=/bin/bash -c "source /opt/ros/humble/setup.bash"

   # Main command
   ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/AI_secretary_robot/install/setup.bash && ros2 launch host_bringup jetrover_host_main.launch.py slam_mode:=localization"

   # Restart policy
   Restart=on-failure
   RestartSec=10

   # Logging
   StandardOutput=journal
   StandardError=journal
   SyslogIdentifier=jetrover-host

   [Install]
   WantedBy=multi-user.target
   ```

2. [Block B] 설치 스크립트:

   **install_service.sh** (/src/bringup/host_bringup/scripts/install_service.sh):

   ```bash
   #!/bin/bash
   # Install JetRover systemd service

   set -e

   SERVICE_FILE="/home/ubuntu/AI_secretary_robot/src/bringup/host_bringup/systemd/jetrover-host.service"
   SYSTEMD_DIR="/etc/systemd/system"

   echo "Installing JetRover Host systemd service..."

   # Copy service file
   sudo cp $SERVICE_FILE $SYSTEMD_DIR/jetrover-host.service

   # Reload systemd
   sudo systemctl daemon-reload

   # Enable service (start on boot)
   sudo systemctl enable jetrover-host.service

   echo ""
   echo "Service installed successfully!"
   echo ""
   echo "Commands:"
   echo "  sudo systemctl start jetrover-host    # Start service"
   echo "  sudo systemctl stop jetrover-host     # Stop service"
   echo "  sudo systemctl status jetrover-host   # Check status"
   echo "  sudo systemctl restart jetrover-host  # Restart service"
   echo "  sudo journalctl -u jetrover-host -f   # View logs"
   echo ""
   echo "Service will auto-start on next boot."
   ```

   ```bash
   chmod +x /home/ubuntu/AI_secretary_robot/src/bringup/host_bringup/scripts/install_service.sh
   ```

3. [Block C] udev Rules (하드웨어 권한):

   **99-jetrover.rules** (/src/bringup/host_bringup/udev/99-jetrover.rules):

   ```
   # JetRover udev rules
   # Automatically set permissions for hardware devices

   # RPLIDAR A1 (LiDAR)
   KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"

   # Jetson UART (Motor Controller)
   KERNEL=="ttyTHS*", MODE="0666"

   # Orbbec Camera (USB)
   SUBSYSTEM=="usb", ATTRS{idVendor}=="2bc5", MODE="0666"

   # I2C (IMU)
   KERNEL=="i2c-[0-9]*", MODE="0666"
   ```

   설치:
   ```bash
   sudo cp /home/ubuntu/AI_secretary_robot/src/bringup/host_bringup/udev/99-jetrover.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. [Block D] 시작/정지 스크립트:

   **service_control.sh**:
   ```bash
   #!/bin/bash
   # Control JetRover Host service

   case "$1" in
       start)
           echo "Starting JetRover Host..."
           sudo systemctl start jetrover-host
           ;;
       stop)
           echo "Stopping JetRover Host..."
           sudo systemctl stop jetrover-host
           ;;
       restart)
           echo "Restarting JetRover Host..."
           sudo systemctl restart jetrover-host
           ;;
       status)
           sudo systemctl status jetrover-host
           ;;
       logs)
           sudo journalctl -u jetrover-host -f
           ;;
       enable)
           echo "Enabling auto-start on boot..."
           sudo systemctl enable jetrover-host
           ;;
       disable)
           echo "Disabling auto-start..."
           sudo systemctl disable jetrover-host
           ;;
       *)
           echo "Usage: $0 {start|stop|restart|status|logs|enable|disable}"
           exit 1
           ;;
   esac
   ```

5. [Block E] CMakeLists.txt 업데이트:

   ```cmake
   install(DIRECTORY
     systemd
     udev
     DESTINATION share/${PROJECT_NAME}/
   )

   install(PROGRAMS
     scripts/install_service.sh
     scripts/service_control.sh
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

6. [Block F] 사용법:

   ```bash
   # 1. 서비스 설치
   ros2 run host_bringup install_service.sh

   # 2. 서비스 시작
   sudo systemctl start jetrover-host

   # 3. 로그 확인
   sudo journalctl -u jetrover-host -f

   # 4. 부팅 시 자동 시작 활성화
   sudo systemctl enable jetrover-host

   # 5. 재부팅 후 확인
   sudo reboot
   # ... 재부팅 후 ...
   sudo systemctl status jetrover-host
   ```

[Constraints]
- User: ubuntu (sudo 없이 GPIO/I2C 접근 필요)
- udev rules로 권한 자동 설정
- systemd 재시작 정책: on-failure (안정성)

[Verification]
sudo systemctl status jetrover-host
sudo journalctl -u jetrover-host --no-pager -n 50

[Output]
- jetrover-host.service (systemd 서비스)
- install_service.sh (설치 스크립트)
- 99-jetrover.rules (udev 규칙)
- service_control.sh (제어 스크립트)
- 부팅 시 자동 시작
```

---

## 🔧 실행 순서 요약

1. **Prompt H1**: Nav2 설치 및 설정 → `src/navigation/nav2_config/`
2. **Prompt H2**: SLAM Toolbox 설정 → `src/navigation/slam_config/`
3. **Prompt H3**: 통합 Bringup 시스템 → `src/bringup/host_bringup/`
4. **Prompt H4**: E-Stop 시스템 → `ros_robot_controller_cpp` 수정
5. **Prompt H5**: TF Tree 완성 → `static_transforms.launch.py`
6. **Prompt H6**: systemd 자동화 → `jetrover-host.service`

**최종 실행:**
```bash
# 빌드
cd /home/ubuntu/AI_secretary_robot
colcon build

# 수동 실행
ros2 launch host_bringup jetrover_host_main.launch.py

# 또는 systemd 서비스로 실행
sudo systemctl start jetrover-host
```

---

## 📝 주의사항

### 하드웨어 연결 확인
프롬프트 실행 전에:
1. LiDAR: `/dev/ttyUSB0` 연결 확인
2. Motor: `/dev/ttyTHS0` 권한 확인
3. Camera: `ls /dev/video*` 확인
4. IMU: `i2cdetect -y -r 1` (주소 0x68 확인)

### TF Tree 검증
모든 프롬프트 완료 후:
```bash
ros2 run tf2_tools view_frames
# frames_*.pdf 파일 확인
# map → odom → base_link 연결 확인
```

### Nav2 튜닝
메카넘 휠 특성에 맞게:
- `max_vel_y`: 좌우 strafe 속도 조정
- `inflation_radius`: 로봇 크기에 맞게
- `controller_frequency`: 20Hz 유지 (50ms)

### 메모리 사용량
Host 프로세스 예상 메모리:
- ROS2 기본: 500MB
- Nav2: 800MB
- SLAM: 1000MB
- 센서 드라이버: 300MB
- **총: ~2.6GB** (8GB 중 충분)

# host_bringup

Unified launch package for JetRover host-side runtime.

## Quick Start

Build and source:

```bash
colcon build --packages-select host_bringup nav2_config slam_config
source install/setup.bash
```

Mapping mode (SLAM only, Nav2 off):

```bash
ros2 launch host_bringup host_bringup_main.launch.py \
  use_slam:=true use_localization:=false use_nav2:=false
```

Localization + Nav2 mode:

```bash
ros2 launch host_bringup host_bringup_main.launch.py \
  use_slam:=true use_localization:=true use_nav2:=true
```

## Map Save Workflow

Save both map formats with one command:

```bash
bash src/navigation/slam_config/scripts/save_map.sh
```

This script updates:
- `src/navigation/slam_config/maps/saved_map.posegraph` (SLAM localization)
- `src/navigation/nav2_config/maps/default_map.yaml` (Nav2 map server)

## Main Launch Arguments

- `use_slam`: Enable SLAM mapping/localization phase.
- `use_localization`: `false`=mapping, `true`=localization.
- `use_nav2`: Enable Nav2 only in localization mode.
- `map_file`: SLAM Toolbox `.posegraph` path for localization mode.
- `nav2_map_file`: Nav2 `.yaml` map path.
- `xacro_path`: Robot xacro used by `robot_state_publisher`.

## System Monitor

`host_system_monitor.py` publishes:
- `/host/system_status` (`std_msgs/String`)
- `/host/system_ok` (`std_msgs/Bool`)

Checks include:
- Required topics (`/scan`, `/odom`, `/imu/data`)
- TF links (`odom->base_link`, `base_link->laser_frame`)
- Nav2 lifecycle state via `*/get_state` services
- Battery freshness/percentage from `/battery_node/state`

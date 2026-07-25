# GLAD Simulation Mapping

ROS 2 simulation mapping package wrapping **slam_toolbox** for laser-based SLAM in simulation.

## Overview

Starts a `slam_toolbox` node configured for online mapping. It subscribes to filtered laser scans and odometry, performs scan matching and loop closure via Ceres solver, and publishes an occupancy grid map.

**Topics subscribed:**

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Filtered laser scan (544 beams, front-facing) |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree (`odom → base_link`) |

**Topics published:**

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid map |
| `/map_metadata` | `nav_msgs/MapMetaData` | Map metadata |
| `/tf` | `tf2_msgs/TFMessage` | Transform `map → odom` |

## Launch

```
ros2 launch glad_simulation_mapping slam_toolbox_launch.py
```

## Dependencies

- ROS 2 (`rclpy`, `launch_ros`)
- `sensor_msgs`, `nav_msgs`
- `slam_toolbox` (binary, not included in this package)

## Build

```bash
colcon build --packages-select glad_simulation_mapping
source install/setup.bash
```

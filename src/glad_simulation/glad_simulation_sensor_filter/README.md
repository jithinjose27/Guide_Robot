# GLAD Sensor Filter

ROS 2 sensor fusion package using **robot_localization**'s Extended Kalman Filter (EKF) to combine odometry and IMU data into a single filtered state estimate.

## Overview

Launches an `ekf_node` that fuses three sensor sources to produce a smooth, drift-corrected odometry estimate at 20 Hz.

**Sensor sources fused:**

| Source | Topic | Type | What's used |
|--------|-------|------|-------------|
| RF2O laser odometry | `/odom_rf2o` | `nav_msgs/Odometry` | Linear velocity X only |
| Motor encoders | `/motor/odom` (remapped from `odom`) | `nav_msgs/Odometry` | Position X, Y (currently disabled in config) |
| IMU orientation | `/imu/data` | `sensor_msgs/Imu` | Orientation Z (yaw) only |

**Topic published:**

| Topic | Type | Description |
|-------|------|-------------|
| `odom` (remapped from `odometry/filtered`) | `nav_msgs/Odometry` | Filtered odometry (pose + twist with covariance) |
| `/tf` | `tf2_msgs/TFMessage` | Transform `odom → base_link` |

## EKF Configuration

All parameters in `config/glad_ekf_params.yaml`.

### General

| Parameter | Value | Description |
|-----------|-------|-------------|
| `frequency` | 20.0 | EKF update rate (Hz) |
| `two_d_mode` | true | 2D mode (z, roll, pitch ignored) |
| `publish_tf` | true | Broadcast `odom → base_link` |
| `world_frame` | `odom` | World frame for the filter |

### Sensor Inputs

#### Source 1 — `/odom_rf2o` (Laser Odometry)

Reads `linear_velocity.x` from the RF2O laser odometry node:

```
odom0_config: X: ✗  Y: ✗  Z: ✗     (position)
              roll: ✗  pitch: ✗  yaw: ✗
              vX: ✓  vY: ✗  vZ: ✗   (velocity)
              roll_dot: ✗  pitch_dot: ✗  yaw_dot: ✗
```

- `differential`: false
- `relative`: false

#### Source 2 — `/motor/odom` (Wheel Odometry)

Motor encoder odometry from `glad_motor_controller`. Currently all states are disabled (not fused), set up as a placeholder for future use.

#### Source 3 — `/imu/data` (IMU Orientation)

Reads `orientation.z` (yaw) from the Madgwick-filtered IMU:

```
imu0_config: X: ✗  Y: ✗  Z: ✗
             roll: ✗  pitch: ✗  yaw: ✓
             vX: ✗  vY: ✗  vZ: ✗
             roll_dot: ✗  pitch_dot: ✗  yaw_dot: ✗
```

- `relative`: true (treats yaw as relative delta rather than absolute heading)

## Launch

```
ros2 launch glad_sensor_filter glad_ekf_launch.py
```

The launch file remaps `odometry/filtered` → `odom` so downstream nodes (Nav2, etc.) receive the filtered estimate as the primary odometry topic. The `glad_sensor_filter` node itself is a lightweight wrapper that declares compatible parameters — the actual EKF logic runs inside the `robot_localization::ekf_node`.

## Pipeline Position

```
RF2O Laser Odometry    Motor Encoder Odometry    Madgwick IMU
  │                        │                        │
  │ /odom_rf2o             │ /motor/odom            │ /imu/data
  ▼                        ▼                        ▼
┌──────────────────────────────────────────────────────┐
│              robot_localization EKF                  │
│  ─► fuses vX from laser odometry                     │
│  ─► fuses yaw from IMU orientation                   │
│  ─► produces filtered pose + twist                   │
└──────────────────────┬───────────────────────────────┘
                       │ odom (remapped)
                       ▼
              Nav2, AMCL, etc.
```

## Dependencies

- ROS 2 (`rclpy`, `launch_ros`)
- `robot_localization`
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`

## Build

```bash
colcon build --packages-select glad_sensor_filter
source install/setup.bash
```

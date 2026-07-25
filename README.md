# G.L.A.D — Guide Lab Assistance Droid

> **Abstract**—To address the limitations of manual laboratory tours, this paper presents G.L.A.D. (Guide Lab Assistance Droid), an autonomous mobile robot designed for interactive, location-specific visitor engagement. Utilizing ROS 2 for navigation and Google Gemini for dynamic natural language generation, the system follows predefined paths to three designated stations within the Robotics and Automation department: a general laboratory introduction zone, the KUKA industrial robot, and the adjacent Automation Laboratory. Upon arrival, G.L.A.D. provides a context-aware, location-specific explanation of each station synthesized via generative AI. To ensure clear communication, the system follows a sequential interaction protocol: it delivers a full technical briefing before transitioning to an interactive Q&A mode, where it processes and responds to individual visitor inquiries one at a time. Built on a Raspberry Pi 4B with LiDAR and IMU integration, this project demonstrates a robust framework for automated educational outreach that balances structured information delivery with responsive AI interaction.

---

**This is the top-level overview of all ROS 2 packages in the G.L.A.D project. Each subdirectory under `src/` is a ROS 2 package (or group of related packages) with its own `README.md` containing detailed documentation — refer to those for configuration, topics, parameters, and usage.**

---

## Package Overview

| Directory | Description |
|-----------|-------------|
| `glad_imu/` | BNO055 IMU hardware driver (`glad_imu_publisher`) and Madgwick orientation filter (`glad_imu_filter`). Publishes raw IMU data and fused orientation. |
| `glad_lidar/` | RPLidar serial driver (`glad_rplidar`), scan filter + resampler (`glad_laser_filter`), and range-flow laser odometry (`rf2o_laser_odometry`). |
| `glad_mapping/` | SLAM wrapper around `slam_toolbox` for online laser-based mapping with Ceres solver and loop closure. |
| `glad_maps/` | Pre-built occupancy grid map files (`robotics_floor/`, `jithin_home/`). |
| `glad_motion_control/` | Motion pipeline: keyboard teleop (`glad_teleop`), priority twist multiplexer (`glad_twist_mux`), slew-rate trajectory limiter (`glad_trajectory_controller`), and ESP32 motor driver with encoder odometry (`glad_motor_controller`). |
| `glad_navigation/` | Nav2-based autonomous navigation: AMCL localization, Regulated Pure Pursuit controller, NavFN global planner, costmaps, and recovery behaviors. |
| `glad_sensor_filter/` | Extended Kalman Filter via `robot_localization` fusing laser odometry (velocity) and IMU (yaw) into a single filtered odometry estimate. |
| `glad_static_tf/` | Static transform broadcaster for sensor mount offsets (`base_link` → `laser`, `base_link` → `imu_link`). |
| `glad_tour/` | Autonomous tour guide — navigates predefined waypoints via Nav2 and delivers AI-generated spoken commentary using Gemini + gTTS. |
| `glad_voice/` | Voice-activated AI assistant with wake word detection, Google STT, Gemini streaming responses, and Edge TTS. |
| `glad_simulation/` | Gazebo-based simulation stack for the G.L.A.D robot. Includes URDF/Gazebo integration, curses teleop, twist mux, trajectory controller, EKF sensor fusion, SLAM Toolbox, and Nav2. See [`src/glad_simulation/README.md`](src/glad_simulation/README.md). |
| `start_glad/` | Top-level bringup packages: `glad_bringup` (full system bringup for testing/navigation) and `glad_final_tour` (complete project showcase including the autonomous tour). |
| `x_old_bringup/` | Archived/legacy bringup (kept for reference). |

---

## System Architecture

### Sensor Pipeline

```
RPLidar ──► glad_rplidar ──► /scan_unfiltered ──► glad_laser_filter ──► /scan
                                                                             │
BNO055 ──► glad_imu_publisher ──► /imu/data_raw ──► glad_imu_filter ──► /imu/data
                                  /imu/mag ─────────────────┘                │
                                                                            │
                            ┌───────────────────────────────────────────────┘
                            ▼
rf2o_laser_odometry ──► /odom_rf2o ──┐
                                      ▼
motor encoders ──► /motor/odom ──► glad_ekf (robot_localization) ──► /odom
                                      ▲
                               /imu/data
```

### TF Tree

```
map ──► odom ──► base_link ──┬──► laser       (static, x=0.10, z=0.05)
                              └──► imu_link    (static, x=0.05)
```

- `map → odom`: AMCL (Nav2) or `slam_toolbox`.
- `odom → base_link`: EKF filter.
- `base_link → laser/imu_link`: Static transforms.

### Command & Control Flow

```
Teleop Keyboard ──► /cmd_vel/teleop ──┐  Priority: stop(100) > teleop(70) > nav(40)
                                       ▼
Nav2 ──► /cmd_vel/nav ──► glad_twist_mux ──► /cmd_vel/raw ──► glad_trajectory_controller
                                                                     │
                                                                     ▼ /cmd_vel
                                                          glad_motor_controller ──► ESP32
                                                                     │
                                                                     ▼ /motor/odom
```

### Application Layer

```
glad_tour ──► nav2_simple_commander ──► goal poses → Nav2 → /cmd_vel/nav
              │
              └──► Gemini ──► gTTS ──► USB speaker

glad_voice ──► Wake word → STT → Gemini streaming → Edge TTS → USB speaker
```

---

## Launching the Robot

### Navigation / Testing Mode

```bash
ros2 launch glad_bringup glad_bringup_launch.py
```

Starts all sensors, filtering, odometry, teleop, and Nav2 with a pre-built map.

### SLAM Mode (Build a New Map)

```bash
ros2 launch glad_bringup glad_bringup_launch.py enable_slam:=true
```

Replaces Nav2 with `slam_toolbox` for online mapping.

### Final Tour Showcase

```bash
ros2 launch glad_final_tour glad_final_tour_launch.py
```

Extends bringup with the autonomous tour guide — navigates waypoints with AI commentary.

---

## Build

```bash
colcon build
source install/setup.bash
```

---

## Details

For per-package details on topics, parameters, launch arguments, configuration files, and dependencies, see the `README.md` inside each package directory.

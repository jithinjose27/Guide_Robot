# G.L.A.D — Guide Lab Assistance Droid

> **Abstract**—To address the limitations of manual laboratory tours, this paper presents G.L.A.D. (Guide Lab Assistance Droid), an autonomous mobile robot designed for interactive, location-specific visitor engagement. Utilizing ROS 2 for navigation and Google Gemini for dynamic natural language generation, the system follows predefined paths to three designated stations within the Robotics and Automation department: a general laboratory introduction zone, the KUKA industrial robot, and the adjacent Automation Laboratory. Upon arrival, G.L.A.D. provides a context-aware, location-specific explanation of each station synthesized via generative AI. To ensure clear communication, the system follows a sequential interaction protocol: it delivers a full technical briefing before transitioning to an interactive Q&A mode, where it processes and responds to individual visitor inquiries one at a time. Built on a Raspberry Pi 4B with LiDAR and IMU integration, this project demonstrates a robust framework for automated educational outreach that balances structured information delivery with responsive AI interaction.

---

**This is the top-level overview of all ROS 2 packages in the G.L.A.D project, modernized for ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS. Each subdirectory under `src/` is a ROS 2 package (or group of related packages) with its own comprehensive, teaching-focused `README.md` containing detailed documentation — refer to those for configuration, circuit pinouts, mathematical foundations, topics, and usage.**

---

## 🛠️ System Prerequisites & Installation

### Environment
- **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)
- **ROS Distribution:** ROS 2 Jazzy Jalisco
- **Python Version:** Python 3.12

### 1. Install ROS 2 Jazzy System Dependencies
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-tf2-tools \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  python3-colcon-common-extensions \
  python3-xacro \
  python3-pip \
  mpg123 \
  alsa-utils
```

### 2. Install Python Dependencies
```bash
pip install --break-system-packages --user \
  pyserial \
  numpy \
  google-genai \
  gTTS \
  edge-tts \
  pygame \
  SpeechRecognition
```

---

## 📦 Package Overview

| Directory | Description |
|-----------|-------------|
| [`glad_imu/`](src/glad_imu/README.md) | BNO055 IMU hardware driver (`glad_imu_publisher`) and Madgwick 6-DoF orientation filter (`glad_imu_filter`). Publishes raw IMU data and fused orientation with continuous sampling. |
| [`glad_lidar/`](src/glad_lidar/README.md) | Official protocol RPLidar serial driver (`glad_rplidar`), scan range filter + uniform 360-bin resampler (`glad_laser_filter`), and range-flow laser odometry (`rf2o_laser_odometry`). |
| [`glad_mapping/`](src/glad_mapping/README.md) | SLAM wrapper around `slam_toolbox` for online laser-based mapping with Ceres non-linear optimization and loop closure. |
| [`glad_maps/`](src/glad_maps/README.md) | Portable ROS 2 package containing pre-built occupancy grid map files (`robotics_floor/`, `jithin_home/`). |
| [`glad_motion_control/`](src/glad_motion_control/README.md) | Motion pipeline: keyboard teleop (`glad_teleop`), priority twist multiplexer (`glad_twist_mux`), slew-rate acceleration limiter (`glad_trajectory_controller`), and ESP32 motor driver with odometry (`glad_motor_controller`). |
| [`glad_navigation/`](src/glad_navigation/README.md) | Nav2-based autonomous navigation: AMCL localization, Regulated Pure Pursuit controller, Navfn global planner, costmaps, and recovery behaviors updated to ROS 2 Jazzy standards. |
| [`glad_sensor_filter/`](src/glad_sensor_filter/README.md) | Extended Kalman Filter fusing laser odometry (translational velocity), wheel encoders, and IMU (heading/yaw) into a filtered odometry estimate with dual-mode support (built-in Python EKF or `robot_localization`). |
| [`glad_static_tf/`](src/glad_static_tf/README.md) | Static transform broadcaster for sensor mount offsets (`base_link` → `laser`, `base_link` → `imu_link`) with configurable launch parameters. |
| [`glad_tour/`](src/glad_tour/README.md) | Autonomous tour guide — navigates predefined waypoints via Nav2 and delivers AI-generated spoken commentary using Gemini + gTTS with offline station fallback. |
| [`glad_voice/`](src/glad_voice/README.md) | Multi-threaded voice-activated AI assistant with wake word detection, Google STT, Gemini streaming responses, Edge TTS, and offline fallback responses. |
| [`glad_simulation/`](src/glad_simulation/README.md) | Gazebo Harmonic (`gz-sim`) digital twin simulation stack for G.L.A.D. Includes URDF/Xacro, Gazebo Sim plugins, curses teleop, twist mux, trajectory controller, EKF, SLAM Toolbox, and Nav2. |
| [`start_glad/`](src/start_glad/README.md) | Top-level bringup packages: `glad_bringup` (full system bringup for testing/navigation) and `glad_final_tour` (complete project showcase including the autonomous tour). |


---

## 🏗️ System Architecture

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
motor encoders ──► /motor/odom ──► glad_ekf (fused state) ─────────────► /odom
                                      ▲
                               /imu/data
```

### Coordinate Frames (TF Tree)

```
map ──► odom ──► base_link ──┬──► laser       (static: x=0.10, z=0.05)
                              └──► imu_link    (static: x=0.05, z=0.00)
```

- `map → odom`: Published by AMCL (Nav2) during navigation or `slam_toolbox` during mapping.
- `odom → base_link`: Published continuously by the Extended Kalman Filter (`glad_ekf_node` / `robot_localization`).
- `base_link → laser / imu_link`: Published by `glad_static_tf`.

### Command & Velocity Flow

```
Teleop Keyboard ──► /cmd_vel/teleop ──┐  Priority: stop(100) > teleop(70) > nav(40)
                                       ▼
Nav2 ──► /cmd_vel/nav ──► glad_twist_mux ──► /cmd_vel/raw ──► glad_trajectory_controller
                                                                     │
                                                                     ▼ /cmd_vel
                                                          glad_motor_controller ──► ESP32 (UART)
                                                                     │
                                                                     ▼ /motor/odom
```

### Application Layer

```
glad_tour ──► BasicNavigator (Nav2) ──► Waypoints → Nav2 → /cmd_vel/nav
              │
              └──► Google Gemini ──► gTTS / mpg123 ──► USB Speaker

glad_voice ──► Microphone → SpeechRecognition → Google Gemini → Edge TTS ──► USB Speaker
```

---

## 🚀 Launching the Robot

### 1. Build the Workspace
```bash
cd ~/Guide_Robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Autonomous Navigation Mode (Pre-built Map)
Starts all sensors, sensor fusion, motor controllers, and the full Nav2 stack using the portable `glad_maps` share path:
```bash
ros2 launch glad_bringup glad_bringup_launch.py
```

### 3. SLAM Mode (Generate a New Map)
Replaces Nav2 with `slam_toolbox` for real-time laser SLAM:
```bash
ros2 launch glad_bringup glad_bringup_launch.py enable_slam:=true
```

To save the completed map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_new_map
```

### 4. Full Autonomous Tour Showcase
Launches the entire robot stack and triggers the autonomous multi-station tour with AI commentary:
```bash
# Optional: export your Gemini API key for dynamic generation
export GEMINI_API_KEY="your_api_key_here"

ros2 launch glad_final_tour glad_final_tour_launch.py
```
*(If no API key or internet is available, G.L.A.D automatically falls back to its built-in offline commentary scripts without failing).*

---

## 🎮 Virtual Simulation (Gazebo Harmonic)

Want to test G.L.A.D in simulation without the physical robot?

```bash
# Launch Gazebo Sim Harmonic with the logistics warehouse:
ros2 launch start_glad_simulation start_glad_simulation.launch.py enable_slam:=true
```

Refer to [`src/glad_simulation/README.md`](src/glad_simulation/README.md) for full simulation details.

---

## 📖 Educational Documentation

Every individual package in this repository includes a dedicated `README.md` formatted like a comprehensive robotics masterclass:
- Explains the **theory and mathematical formulas** (Madgwick filtering, Kalman sensor fusion, range-flow odometry).
- Details **hardware wiring schematics and pinouts** (BNO055 I2C, RPLidar UART, ESP32 motor controller).
- Documents all **published and subscribed ROS 2 topics**, message types, and configurable parameters.
- Outlines **troubleshooting guides** with practical diagnostic steps.

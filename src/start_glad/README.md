# GLAD Start (System Bringup Orchestrator)

Welcome to the **`start_glad`** master orchestrator! When deploying G.L.A.D. on real hardware, you do not need to manually open 10 terminals to launch individual drivers and controllers. Everything is orchestrated here through two primary launch packages:

---

## The Two Master Launch Files

| Launch Package | Launch Command | Purpose |
|---|---|---|
| **`glad_bringup`** | `ros2 launch glad_bringup glad_bringup_launch.py` | **Hardware & Navigation Testing Mode.** Boots all sensors, filtering, odometry, motor control, and Nav2 navigation (or SLAM). **Does NOT start the tour**, leaving the robot stationary and ready for manual teleop driving, sensor diagnostics, and RViz navigation goal testing. |
| **`glad_final_tour`** | `ros2 launch glad_final_tour glad_final_tour_launch.py` | **Main Production Showcase.** This is the **only file you need to launch** to execute the complete end-to-end task. Boots the entire robot and automatically launches `glad_tour` at $t=10\text{ s}$ to guide visitors through all stations with AI commentary. |

---

## Why Are Launch Times Sequenced?

In complex mobile robotics systems, starting 15 nodes simultaneously at timestamp $t = 0$ leads to race conditions:
1. The laser filter tries to subscribe to `/scan_unfiltered` before the RPLidar serial connection is established.
2. The EKF tries to fuse odometry before the IMU filter computes its first orientation quaternion.
3. Nav2 costmap layers fail to initialize because the continuous transform `odom → base_link` is not yet being broadcast.

To ensure deterministic, error-free startup, `start_glad` staggers subsystem initialization:

| Timestamp | Subsystem | Action |
|---|---|---|
| **$t = 0.0\text{ s}$** | `glad_static_tf` & `glad_rplidar` | Mounting transforms broadcast; RPLidar serial port opens and motor spins up. |
| **$t = 1.0\text{ s}$** | `glad_laser_filter` | Laser scan filtering and uniform 360-bin resampler start publishing `/scan`. |
| **$t = 1.5\text{ s}$** | `glad_imu_publisher` | BNO055 I2C driver starts publishing `/imu/data_raw` and `/imu/mag`. |
| **$t = 2.0\text{ s}$** | `glad_imu_filter` | Madgwick orientation filter begins publishing fused `/imu/data`. |
| **$t = 2.5\text{ s}$** | `rf2o_laser_odometry` | SVD scan matcher starts computing planar velocity on `/odom_rf2o`. |
| **$t = 3.0\text{ s}$** | `glad_sensor_filter` | 2D EKF begins fusing odometry sources and broadcasting `odom → base_link`. |
| **$t = 3.5\text{ s}$** | `glad_mapping` | *(When `enable_slam:=true`)* SLAM Toolbox begins building an online map. |
| **$t = 4.0\text{ s}$** | `glad_teleop` | Twist multiplexer, trajectory smoother, and ESP32 motor bridge activate. |
| **$t = 6.0\text{ s}$** | `glad_navigation` | *(When `enable_slam:=false`)* AMCL localization and Nav2 lifecycle nodes transition to active. |
| **$t = 10.0\text{ s}$** | `glad_tour` | *(Only in `glad_final_tour`)* Autonomous tour guide begins navigating waypoints! |
| **$t = 11.0\text{ s}$** | `glad_voice` | *(Optional when `enable_voice:=true`)* Interactive voice assistant activates for visitor Q&A. |

---

## Usage Instructions

### 1. Navigation & Hardware Testing (Everything except Tour)
Use this command while developing, tuning costmaps, driving with the keyboard teleop, or testing Nav2 waypoint goals via RViz:

```bash
ros2 launch glad_bringup glad_bringup_launch.py
```

To run keyboard teleoperation in your terminal during bringup:
```bash
ros2 run glad_teleop teleop_node
```

### 2. SLAM Mapping Mode (Build a New Map)
Drive around the facility to map the environment using `slam_toolbox`:

```bash
ros2 launch glad_bringup glad_bringup_launch.py enable_slam:=true
```

Save your map once finished:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_new_map
```

### 3. Final Production Showcase (Main Robot Task)
This is the single primary command for demonstrations and official runs. The robot powers up all sensors, initializes Nav2, and embarks on the tour:

```bash
# Export your Gemini API key (optional: if omitted, high-quality offline scripts are used)
export GEMINI_API_KEY="your-api-key-here"

ros2 launch glad_final_tour glad_final_tour_launch.py
```

*To also enable the background voice assistant for interactive Q&A:*
```bash
ros2 launch glad_final_tour glad_final_tour_launch.py enable_voice:=true
```

---

## Building the Bringup Packages

```bash
cd ~/Guide_Robot
source /opt/ros/jazzy/setup.bash
colcon build --packages-select glad_bringup glad_final_tour
source install/setup.bash
```

# GLAD Start (System Bringup) 🏁

Welcome to the **`start_glad`** master orchestrator! If you want to run the whole robot with a single terminal command, this is where it happens.

---

## 🚀 The Two Master Launch Files

| Launch Package | When to Use It |
|---|---|
| **`glad_bringup`** | Use when setting up, testing navigation, driving via teleop, or building a new map with SLAM. |
| **`glad_final_tour`** | Use for the complete grand showcase: starts all systems and immediately commands the autonomous AI tour guide! |

---

## ⏱️ Why Are Launch Times Sequenced?

If you launch 15 complex ROS 2 nodes simultaneously at timestamp $t = 0$:
1. The laser filter tries to subscribe to `/scan_unfiltered` before the RPLidar serial port opens.
2. The EKF tries to fuse odometry before the IMU filter computes its first orientation quaternion.
3. Nav2 costmaps fail to load because TF transforms (`odom → base_link`) aren't published yet.

To prevent race conditions, `start_glad` orchestrates a smooth, bulletproof launch sequence:

| Time | Package | What Gets Activated |
|---|---|---|
| **$t = 0.0\text{ s}$** | `glad_static_tf` & `glad_rplidar` | Mounting transforms broadcast; RPLidar motor spins up. |
| **$t = 1.0\text{ s}$** | `glad_laser_filter` | Laser scan filtering and FOV cropping begins. |
| **$t = 1.5\text{ s}$** | `glad_imu_publisher` | BNO055 $I^2C$ sensor driver starts publishing raw vectors. |
| **$t = 2.0\text{ s}$** | `glad_imu_filter` | Madgwick orientation filter begins publishing `/imu/data`. |
| **$t = 2.5\text{ s}$** | `rf2o_laser_odometry` | SVD scan matcher starts computing planar velocity. |
| **$t = 3.0\text{ s}$** | `glad_sensor_filter` | EKF begins fusing odometry and broadcasting `odom → base_link`. |
| **$t = 3.5\text{ s}$** | `glad_mapping` | *(Only in SLAM mode)* SLAM Toolbox begins building a live map. |
| **$t = 4.0\text{ s}$** | `glad_teleop` | Twist mux, trajectory smoother, and motor bridge activate. |
| **$t = 6.0\text{ s}$** | `glad_navigation` | *(Only in Localization mode)* AMCL and Nav2 servers initialize. |
| **$t = 10.0\text{ s}$** | `glad_tour` | *(In `glad_final_tour`)* Autonomous tour guide begins navigating! |

---

## 🛠️ Usage Commands

### 1. Standard Autonomous Navigation (Default Pre-built Map)
```bash
ros2 launch glad_bringup glad_bringup_launch.py
```

### 2. SLAM Mapping Mode (Build a New Map)
```bash
ros2 launch glad_bringup glad_bringup_launch.py enable_slam:=true
```

### 3. Full AI Autonomous Tour Showcase
```bash
ros2 launch glad_final_tour glad_final_tour_launch.py
```

---

## 📦 Building

```bash
colcon build --packages-select glad_bringup glad_final_tour
source install/setup.bash
```

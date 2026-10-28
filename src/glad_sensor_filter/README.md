# GLAD Sensor Filter (EKF) 🛰️

Welcome to the **`glad_sensor_filter`** package! This is the state estimation core of the robot. It takes noisy, imperfect readings from multiple different sensors and fuses them into a single, crystal-clear, rock-steady estimate of the robot's position and velocity.

---

## 🧭 Why Do We Need Sensor Fusion?

No single sensor on a mobile robot is perfect:
- **Wheel Encoders:** Great for tracking short-distance wheel rotation, but when wheels slip on smooth floors or bump over door sills, the robot thinks it is moving when it isn't.
- **Laser Odometry (`rf2o`):** Very accurate in clutter-rich rooms by matching wall returns, but can drift or lose lock in long empty corridors or when people walk right in front of the scanner.
- **IMU (Inertial Measurement Unit):** Fantastic at measuring rapid angular turns ($\omega_z$), but cannot directly measure forward position ($x, y$).

**Sensor fusion** combines the best strengths of each sensor while mathematically canceling out their individual weaknesses!

---

## 🔄 The Data Flow Architecture

```
 Laser Odometry         Wheel Encoders               Madgwick IMU
  (/odom_rf2o)           (/motor/odom)                (/imu/data)
       │                       │                           │
       │ vx, vy                │ vx (fallback)             │ yaw, vyaw (angular rate)
       └───────────────┬───────┴───────────────────────────┘
                       ▼
       ┌───────────────────────────────┐
       │     Extended Kalman Filter    │
       │  (glad_ekf_node / ekf_node)   │
       │  ─► Integrates planar motion  │
       │  ─► Fuses IMU heading         │
       │  ─► Eliminates drift          │
       └───────────────┬───────────────┘
                       │
       ┌───────────────┴───────────────┐
       ▼                               ▼
  Topic: /odom                    TF Transform:
(nav_msgs/Odometry)             odom ──► base_link
       │
       ▼
 Nav2, AMCL, SLAM
```

---

## ⚙️ Dual-Mode Flexibility: Python or C++

To ensure **everyone can run this project effortlessly on any machine**, this package supports two complementary execution engines:

1. **Built-in Python 2D EKF (`glad_ekf_node`):**
   - Active by default.
   - Requires **zero external C++ dependencies** beyond standard ROS 2.
   - Runs a lightweight kinematic prediction-correction filter at 20 Hz.
   - Broadcasts the continuous `odom → base_link` TF transform.
2. **Industrial C++ EKF (`robot_localization`):**
   - Can be activated whenever `ros-jazzy-robot-localization` is installed.
   - Full 15-state covariance-based nonlinear Extended Kalman Filter.
   - Enabled with a simple launch flag: `use_robot_localization:=true`.

---

## 🛠️ Configuration (`config/glad_ekf_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `frequency` | `20.0 Hz` | EKF filter update rate. |
| `two_d_mode` | `true` | Planar ground robot mode ($z$, roll, pitch held constant). |
| `publish_tf` | `true` | Broadcasts `odom → base_link` transform. |
| `odom_frame` | `odom` | The continuous odometric reference frame. |
| `base_link_frame` | `base_link` | The robot chassis reference frame. |

---

## 🚀 Building & Launching

```bash
colcon build --packages-select glad_sensor_filter
source install/setup.bash
```

### Launch in Built-in Mode (Zero extra dependencies):
```bash
ros2 launch glad_sensor_filter glad_ekf_launch.py
```

### Launch with Industrial `robot_localization` C++ node:
```bash
# If installed via: sudo apt install ros-jazzy-robot-localization
ros2 launch glad_sensor_filter glad_ekf_launch.py use_robot_localization:=true
```

### Verify Output:
```bash
# View filtered odometry messages
ros2 topic echo /odom

# View the transform connection
ros2 run tf2_ros tf2_echo odom base_link
```

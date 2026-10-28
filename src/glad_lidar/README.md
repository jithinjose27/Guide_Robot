# GLAD LiDAR

Welcome to the **`glad_lidar`** sensor pipeline! If the wheels are the legs of the G.L.A.D robot, the LiDAR is its eyes. This directory contains everything needed to interface with physical 2D laser scanners, clean raw beam returns, and calculate smooth planar odometry purely from laser scan changes.

---

## Architecture & Data Flow

```
┌────────────────────────────────────────────────────────┐
│                   RPLidar Sensor                       │
│     (Spinning 360° laser rangefinder @ 5.5 - 10 Hz)    │
└──────────────────────────┬─────────────────────────────┘
                           │ UART Serial (/dev/ttyUSB0)
                           ▼
┌────────────────────────────────────────────────────────┐
│           Package 1: glad_rplidar                      │
│  - Decodes 5-byte Slamtec serial packets               │
│  - Reconstructs complete 360° revolutions              │
│  - Bins into uniform angular grid                      │
└──────────────────────────┬─────────────────────────────┘
                           │ Topic: /scan_unfiltered (sensor_msgs/LaserScan)
                           ▼
┌────────────────────────────────────────────────────────┐
│         Package 2: glad_laser_filter                   │
│  - Crops angular FOV to keep front 180° [-90°, +90°]   │
│  - Clips out-of-bounds distances to +inf               │
│  - Resamples to exact beam counts with angle fidelity  │
└──────────────────────────┬─────────────────────────────┘
                           │ Topic: /scan (sensor_msgs/LaserScan)
              ┌────────────┴────────────┐
              ▼                         ▼
┌───────────────────────────┐     ┌──────────────────────┐
│ Package 3:                │     │ SLAM Toolbox & Nav2  │
│ rf2o_laser_odometry       │     │ Obstacle Costmaps    │
│ - SVD / Kabsch alignment  │     └──────────────────────┘
│ - Fuses planar velocities │
└─────────────┬─────────────┘
              │ Topic: /odom_rf2o (nav_msgs/Odometry)
              ▼
   glad_sensor_filter (EKF)
```

---

## Subpackages Detailed

### 1. `glad_rplidar` — Hardware Serial Driver
- **What it does:** Directly talks to SLAMTEC RPLidar sensors (A1, A2, A3).
- **How it works:**
  1. Opens serial connection (default: `/dev/ttyUSB0` at 115200 baud).
  2. Asserts motor power control lines (`dtr=False`).
  3. Decodes standard 5-byte measurement nodes:
     - `Byte 0`: Sync flag and quality score.
     - `Byte 1 & 2`: Angle fractional representation ($q_6$).
     - `Byte 3 & 4`: Measured distance ($q_2$ millimeters).
  4. Detects full 360° revolution boundaries and bins returns into a 360-sample uniform polar grid from $-\pi$ to $+\pi$.
- **Topic Published:** `/scan_unfiltered` (`sensor_msgs/LaserScan`).
- **Run directly:**
  ```bash
  ros2 run glad_rplidar rplidar_node
  ```

### 2. `glad_laser_filter` — Scan Filter & Resampler
- **What it does:** Cleans and shapes the raw scan before sending it to navigation and SLAM algorithms.
- **Why it matters:**
  - In differential drive robots, the rear of the LiDAR might see the robot's own chassis, antenna, or wiring harness.
  - Costmap raytracing algorithms crash if passed negative infinity (`-inf`).
  - This node filters out non-forward angles and clips close-range noise ($< 0.20\text{ m}$) safely to positive infinity (`+inf`).
- **Topic Subscribed:** `/scan_unfiltered`
- **Topic Published:** `/scan`
- **Run directly:**
  ```bash
  ros2 run glad_laser_filter laser_filter_node
  ```

### 3. `rf2o_laser_odometry` — Planar Laser Odometry
- **What it does:** Computes the robot's displacement ($\Delta x, \Delta y, \Delta \theta$) and linear/angular velocities between consecutive laser scans without relying on wheel encoders.
- **Algorithm (Kabsch Alignment via SVD):**
  1. Identifies obstacle points that were observed in both scan $k-1$ and scan $k$.
  2. Computes the centroids and cross-covariance matrix $H$ of corresponding point pairs.
  3. Uses Singular Value Decomposition ($H = U \Sigma V^T$) to extract the optimal rotation matrix $R = V U^T$ and translation vector $T$.
  4. Divides by time step $dt$ to generate true instantaneous velocity ($v_x, v_y, \omega_z$).
- **Topic Published:** `/odom_rf2o` (`nav_msgs/Odometry`).
- **Run directly:**
  ```bash
  ros2 run rf2o_laser_odometry rf2o_odometry_node
  ```

---

## Hardware Setup & Permissions

If you get a `Permission Denied` error when connecting to your LiDAR:

1. Add your Linux user to the `dialout` group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
2. Log out and log back in, or run `newgrp dialout`.
3. Check where your LiDAR is enumerated:
   ```bash
   ls -la /dev/ttyUSB*
   ```
   *(If your LiDAR is on `/dev/ttyUSB1`, simply update `config/rplidar_params.yaml` or launch with `serial_port:=/dev/ttyUSB1`)*.

---

## Building & Testing

```bash
colcon build --packages-select glad_rplidar glad_laser_filter rf2o_laser_odometry
source install/setup.bash
```

Launch the entire LiDAR sensing pipeline together:
```bash
ros2 launch glad_rplidar glad_rplidar_launch.py
ros2 launch glad_laser_filter glad_laser_filter_launch.py
ros2 launch rf2o_laser_odometry rf2o_laser_odometry_launch.py
```

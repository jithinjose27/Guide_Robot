# GLAD IMU

Welcome to the **`glad_imu`** package! An Inertial Measurement Unit (IMU) is the inner ear of the robot — it gives G.L.A.D an intuitive sense of balance, orientation, and angular turning speed even in pitch darkness or wheel-slip conditions.

---

## How Inertial Navigation Works

The G.L.A.D robot uses an **Adafruit BNO055 9-DOF** (Degrees of Freedom) sensor connected via $I^2C$. It contains:
1. **3-Axis Gyroscope:** Measures instantaneous rotational velocity ($\omega_x, \omega_y, \omega_z$ in $\text{rad/s}$). Excellent for fast movements, but drifts over time.
2. **3-Axis Accelerometer:** Measures linear acceleration ($a_x, a_y, a_z$ in $\text{m/s}^2$) including gravity ($9.81\text{ m/s}^2$). When the robot is standing still or moving at constant speed, gravity points straight down, giving an absolute reference for roll and pitch.
3. **3-Axis Magnetometer:** Senses the Earth's ambient magnetic field vector ($\mu\text{T}$) to provide an absolute magnetic heading (yaw).

### The Sensor Pipeline:
```
┌──────────────────────────────────────────────┐
│           Adafruit BNO055 Hardware           │
└──────────────────────┬───────────────────────┘
                       │ I2C Bus (SCL / SDA)
                       ▼
┌──────────────────────────────────────────────┐
│        Package 1: glad_imu_publisher         │
│  - Reads raw gyro, accel, mag                │
│  - Applies calibration bias & noise gate     │
└──────────────┬───────────────────────────────┘
               │ /imu/data_raw & /imu/mag
               ▼
┌──────────────────────────────────────────────┐
│         Package 2: glad_imu_filter           │
│  - Madgwick Gradient-Descent Filter          │
│  - Fuses gyro integration with gravity & mag │
└──────────────┬───────────────────────────────┘
               │ /imu/data (Fused Orientation Quaternion)
               ▼
      glad_sensor_filter (EKF)
```

---

## The Madgwick Filter Algorithm

Why not just integrate gyroscope readings over time ($\theta_{new} = \theta_{old} + \omega \cdot dt$)?
Because tiny sensor biases accumulate every millisecond, causing the robot to believe it is spinning even while stationary!

The **Madgwick AHRS (Attitude and Heading Reference System)** algorithm combines:
- **Fast Gyroscope Integration:** Provides low-latency, dynamic tracking of rapid rotations.
- **Gradient Descent Correction ($\beta$ gain):** Slowly nudges the orientation quaternion towards the gravity vector (from accelerometer) and magnetic north (from magnetometer).

The filter gain $\beta$ balances responsiveness against noise:
- A higher $\beta$ corrects gyro drift faster, but allows mechanical vibrations to jitter the orientation.
- A lower $\beta$ (e.g. $0.05 - 0.1$) gives buttery-smooth heading tracking for mobile robots.

---

## Packages Overview

### 1. `glad_imu_publisher`
- **Driver:** Pure Python driver interfacing with the BNO055.
- **Config:** `config/bno055_config.yaml` stores zero-rate gyro offsets and noise deadzones.
- **Topics Published:**
  - `/imu/data_raw` (`sensor_msgs/Imu`): Raw acceleration and angular velocity.
  - `/imu/mag` (`sensor_msgs/MagneticField`): Calibrated magnetic field vector in Tesla.
- **Run standalone:**
  ```bash
  ros2 run glad_imu_publisher imupub
  ```

### 2. `glad_imu_filter`
- **Filter:** Pure Python implementation of the Madgwick filter.
- **Parameters (`config/madgwick_params.yaml`):**
  - `beta`: Filter correction rate (default: `0.1`).
  - `use_mag`: Incorporate magnetometer (default: `true`).
  - `world_frame`: `"enu"` (East-North-Up coordinate convention).
- **Topic Published:**
  - `/imu/data` (`sensor_msgs/Imu`): Complete IMU message with fused `orientation` quaternion $(x, y, z, w)$ and covariance.
- **Run standalone:**
  ```bash
  ros2 run glad_imu_filter imu_madgwick
  ```

---

## Wiring & Hardware Setup (Raspberry Pi)

| BNO055 Pin | Raspberry Pi Pin | Function |
|---|---|---|
| **VIN** | Pin 1 (3.3V) | Power |
| **GND** | Pin 6 (GND) | Ground |
| **SCL** | Pin 5 (GPIO 3 / SCL1) | $I^2C$ Clock |
| **SDA** | Pin 3 (GPIO 2 / SDA1) | $I^2C$ Data |

### Enable I2C on Ubuntu / Raspberry Pi:
```bash
sudo raspi-config
# Navigate to: 3 Interface Options -> I4 I2C -> Enable -> Yes
```
Verify the device is detected on address `0x28` or `0x29`:
```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

---

## Building & Launching

```bash
colcon build --packages-select glad_imu_publisher glad_imu_filter
source install/setup.bash
```

Launch both driver and filter together:
```bash
ros2 launch glad_imu_publisher imu_publisher_launch.py
```

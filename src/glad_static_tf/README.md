# GLAD Static TF 📐

Welcome to the **`glad_static_tf`** package! Think of this package as the **physical blueprint** of the robot told in the language of coordinate frames.

---

## 🤖 Why Do We Need Static Transforms?

A mobile robot is not a single point in space. It has physical dimensions and components mounted at different locations:
- The **RPLidar** is mounted towards the front top deck so it has an unobstructed view of the room.
- The **BNO055 IMU** is secured on the chassis plate near the center of rotation to minimize artificial centripetal acceleration.
- The **base_link** represents the origin of the robot body (usually centered between the drive wheels at floor or axle height).

When the LiDAR detects an obstacle $1.5\text{ meters}$ directly in front of itself, that obstacle is actually $1.6\text{ meters}$ in front of the robot's center because the LiDAR is mounted $10\text{ cm}$ forward ($x = +0.10\text{ m}$).

Without telling ROS where these sensors are physically bolted to the chassis, your maps will distort, local costmaps will misalign with obstacles, and the robot might bump into door frames! `glad_static_tf` solves this by broadcasting fixed, non-changing coordinate transformations (`tf2_ros::StaticTransformBroadcaster`) from `base_link` to `laser` and `imu_link`.

---

## 🌳 The TF Tree

```
map (World / Navigation Origin)
 └──► odom (Continuous Smooth Odometry)
       └──► base_link (Robot Chassis Reference)
             ├──► laser (Static Mount: x=+0.10 m, z=+0.05 m)
             └──► imu_link (Static Mount: x=+0.05 m, z=0.00 m)
```

- **`map → odom`**: Computed dynamically by AMCL (localization against a map) or SLAM Toolbox.
- **`odom → base_link`**: Broadcast continuously by the EKF (`robot_localization` or `glad_sensor_filter`).
- **`base_link → laser` & `base_link → imu_link`**: Broadcast once by **this package** and latched on `/tf_static`.

---

## 📏 Default Sensor Mounting Offsets

| Transform | X (Forward) | Y (Left) | Z (Up) | Roll / Pitch / Yaw | Physical Meaning |
|---|---|---|---|---|---|
| `base_link → laser` | `+0.10 m` | `0.00 m` | `+0.05 m` | `0° / 0° / 0°` | LiDAR is $10\text{ cm}$ forward and $5\text{ cm}$ above base center. |
| `base_link → imu_link` | `+0.05 m` | `0.00 m` | `0.00 m` | `0° / 0° / 0°` | IMU is $5\text{ cm}$ forward of axle center. |

---

## 🛠️ How to Customize for Your Own Robot

Did you build a custom chassis with different sensor locations? You don't need to touch the Python source code! You can easily pass your custom offsets as parameters:

```bash
ros2 run glad_static_tf static_tf_broadcaster --ros-args \
  -p laser_x:=0.15 \
  -p laser_z:=0.12 \
  -p imu_x:=0.00
```

### Supported Parameters:
- `base_frame_id` (string, default: `"base_link"`)
- `laser_frame_id` (string, default: `"laser"`)
- `laser_x`, `laser_y`, `laser_z` (double, meters)
- `laser_roll`, `laser_pitch`, `laser_yaw` (double, radians)
- `imu_frame_id` (string, default: `"imu_link"`)
- `imu_x`, `imu_y`, `imu_z` (double, meters)
- `imu_roll`, `imu_pitch`, `imu_yaw` (double, radians)

---

## 🚀 Running & Verification

### Launch:
```bash
ros2 launch glad_static_tf glad_static_tf_launch.py
```

### Inspect the TF Tree:
To verify that your transforms are active and connected:
```bash
# Print the transform from base_link to laser
ros2 run tf2_ros tf2_echo base_link laser

# View the full TF graph visually
ros2 run tf2_tools view_frames
evince frames.pdf
```

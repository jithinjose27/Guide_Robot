# 🤖 G.L.A.D. Simulation Suite (ROS 2 Jazzy & Gazebo Harmonic)

Welcome to the digital twin and simulation environment of the **G.L.A.D.** (Generic Logistics Autonomous Device) guide robot!

One of the best principles in modern robotics is: **test in simulation before deploying to hardware**. Whether you don't have the physical robot yet, or you want to experiment with navigation algorithms and safety behaviors without risking wheel stalls or bumper collisions, this simulation suite provides a full virtual twin of G.L.A.D. running on **Ubuntu 24.04 LTS (Noble Numbat)** and **ROS 2 Jazzy Jalisco** using **Gazebo Harmonic (`gz-sim`)**.

---

## 🏗️ Architecture Overview

The simulation stack faithfully reproduces the physical robot's sensor pipeline, kinematics, and control architecture:

```
+-----------------------------------------------------------------------------------+
|                            Gazebo Sim Harmonic (gz-sim)                           |
|  - Differential Drive System (gz-sim-diff-drive-system)                          |
|  - 360° GPU LiDAR (gz-sim-sensors-system / gpu_lidar)                            |
|  - 6-DoF IMU (gz-sim-imu-system / imu_sensor)                                     |
|  - Joint State Publisher (gz-sim-joint-state-publisher-system)                   |
+-----------------------------------------------------------------------------------+
                                        |
                            ros_gz_bridge (Parameter Bridge)
       /clock, /cmd_vel, /odom/wheel, /scan, /imu/raw, /joint_states
                                        |
+-----------------------------------------------------------------------------------+
|                             ROS 2 Jazzy Control Stack                             |
|                                                                                   |
|  [Teleop / Nav2] ──> twist_mux ──> trajectory_controller ──> /cmd_vel             |
|                                                                                   |
|  [/odom/wheel + /imu/raw] ──> glad_ekf_node ──> /odom (odom -> base_link TF)     |
|                                                                                   |
|  [/scan + /odom] ──> slam_toolbox (Mapping) OR Nav2 Stack (Autonomous Nav)        |
+-----------------------------------------------------------------------------------+
```

---

## 📦 Packages in this Directory

| Package | Purpose |
|---|---|
| `start_glad_simulation` | Bringup orchestrator — launches Gazebo worlds, timed node startup, and SLAM/Nav2 mode selection. |
| `glad_simulation_urdf` | URDF/Xacro description, physical inertia and collision models, Gazebo Harmonic system plugins, and RViz2 visualization. |
| `glad_simulation_teleop` | Interactive curses-based terminal teleop with keyboard control, live speed adjustments, and safety emergency stop. |
| `glad_simulation_twist_mux` | Multiplexes velocity command streams with strict priority (`E-STOP` > `Teleop` > `Nav2`). |
| `glad_simulation_trajectory_controller` | Accel/decel velocity ramp filter ensuring smooth, realistic robot acceleration without jerky motion. |
| `glad_simulation_motor_controller` | Odometry bridge relaying Gazebo's `/odom/wheel` to the standardized `/motor/odom` topic. |
| `glad_simulation_sensor_filter` | 2D Extended Kalman Filter fusing simulated wheel odometry and IMU data into smooth, drift-resistant `/odom`. |
| `glad_simulation_mapping` | Online asynchronous SLAM mapping powered by `slam_toolbox`. |
| `glad_simulation_navigation` | Full ROS 2 Jazzy Nav2 stack (AMCL localization, Navfn global planner, Regulated Pure Pursuit local controller, behavior server). |

---

## 🚀 Prerequisites & Installation

To run the simulation on Ubuntu 24.04 with ROS 2 Jazzy:

1. **Install Gazebo Harmonic and ROS GZ integration:**
   ```bash
   sudo apt update
   sudo apt install -y \
     ros-jazzy-ros-gz \
     ros-jazzy-ros-gz-bridge \
     ros-jazzy-ros-gz-interfaces \
     ros-jazzy-navigation2 \
     ros-jazzy-nav2-bringup \
     ros-jazzy-slam-toolbox \
     ros-jazzy-robot-state-publisher \
     ros-jazzy-joint-state-publisher \
     ros-jazzy-rviz2 \
     python3-xacro
   ```

2. **Build the simulation packages:**
   ```bash
   cd ~/Guide_Robot
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## 🎮 Launching the Simulation

### Mode 1: SLAM Mapping Mode (Drive & Build a Map)
Spawn the robot in the virtual warehouse and build a map with your keyboard:

```bash
ros2 launch start_glad_simulation start_glad_simulation.launch.py enable_slam:=true
```

In a second terminal, if you want keyboard control in your active window:
```bash
ros2 run glad_simulation_teleop teleop_node
```

### Mode 2: Autonomous Navigation Mode (Nav2 with Pre-built Map)
Spawn the robot in the warehouse and navigate autonomously using Nav2:

```bash
ros2 launch start_glad_simulation start_glad_simulation.launch.py enable_slam:=false
```
*Note: By default, this automatically loads the included `glad_maps/robotics_floor` map.*

---

## 🕹️ Keyboard Controls

When running `teleop_node`, you can drive G.L.A.D using your keyboard:

```
       Q=↖  W=↑  E=↗
       A=←  S=■  D=→
            X=↓
```

| Key | Action |
|---|---|
| `W` | Drive Forward |
| `S` | Smooth Stop |
| `A` | Spin Left |
| `D` | Spin Right |
| `Q` | Curve Forward-Left |
| `E` | Curve Forward-Right |
| `X` | Reverse |
| `Space` | Emergency Stop (E-Stop toggle) |
| `+` / `-` | Increase / Decrease linear speed limit |
| `]` / `[` | Increase / Decrease angular speed limit |

---

## 🌍 Virtual Worlds Included

1. **`warehouse.world`** (Default): A realistic 20m × 15m logistics warehouse complete with perimeter walls, storage shelf corridors, structural pillars, and open runways.
2. **`empty.world`**: A clean, infinite ground plane with sunlight and physics plugins for basic testing and odometry calibration.

To switch worlds, supply the `world` argument:
```bash
ros2 launch start_glad_simulation start_glad_simulation.launch.py world:=$(ros2 pkg prefix start_glad_simulation)/share/start_glad_simulation/worlds/empty.world
```

---

## 💡 Teaching Tips & Troubleshooting

- **Simulated Time Synchronization (`use_sim_time:=true`)**: In simulation, all nodes must synchronize with Gazebo's `/clock` rather than the computer's system wall clock. The launch files automatically propagate `use_sim_time:=true` to all nodes and the Nav2 lifecycle manager.
- **Headless & Virtual Machines**: If running inside WSL2 or on a server without an OpenGL GPU, you can run Gazebo headless by passing Gazebo server arguments, or launch RViz on your host display.
- **Odometry & EKF Fusion**: The simulation EKF bridges wheel odometry and IMU angular rate at 20 Hz, ensuring that SLAM and Nav2 receive smooth transformation frames (`odom -> base_link`) with valid covariance matrices.

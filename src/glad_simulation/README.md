# G.L.A.D Simulation

ROS 2 simulation stack for the **G.L.A.D** (Generic Logistics Autonomous Device) robot. Launches Gazebo with a differential-drive robot, curses-based teleop, sensor fusion EKF, SLAM, and Nav2.

## Quick Start

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
ros2 launch start_glad_simulation start_glad_simulation.launch.py
```

## Packages

| Package | Purpose |
|---|---|
| `start_glad_simulation` | Bringup orchestrator — timed launch sequence, Gazebo worlds |
| `glad_simulation_urdf` | Robot model (Xacro), Gazebo plugins, robot_state_publisher |
| `glad_simulation_teleop` | Curses-based keyboard teleop (xterm) |
| `glad_simulation_twist_mux` | Velocity source multiplexer (teleop, nav, estop) |
| `glad_simulation_trajectory_controller` | Accel/decel ramping for smooth motion |
| `glad_simulation_motor_controller` | Odometry bridge `/odom/wheel` → `/motor/odom` |
| `glad_simulation_sensor_filter` | EKF sensor fusion (wheel odometry + IMU) |
| `glad_simulation_mapping` | SLAM Toolbox (async mapping) |
| `glad_simulation_navigation` | Nav2 stack (localization, planning, control) |

## Controls

```
       Q=↙  W=↑  E=↘
       A=←  S=■  D=→
            X=↓
```

| Key | Action |
|---|---|
| `W` | Forward |
| `S` | Stop |
| `A` | Turn left |
| `D` | Turn right |
| `Q` | Forward + left curve |
| `E` | Forward + right curve |
| `X` | Reverse |
| `Space` | Emergency stop (toggle) |
| `+` / `-` | Increase / decrease linear speed |
| `]` / `[` | Increase / decrease angular speed |

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `enable_slam` | `true` | Start SLAM (false = use Nav2 with static map) |
| `map_yaml` | `""` | Map YAML path (required when enable_slam:=false) |
| `world` | `warehouse.world` | Gazebo world file |

## Launch Sequence

| Time | Package |
|---|---|
| t=0s | `glad_simulation_urdf` — Gazebo, robot_state_publisher, RViz |
| t=1s | `glad_simulation_teleop` — teleop, twist_mux, trajectory_controller, motor_controller |
| t=2s | `glad_simulation_sensor_filter` — EKF |
| t=3s | `glad_simulation_mapping` (SLAM) or `glad_simulation_navigation` (Nav2) |

## Data Flow

```
Teleop (Q/W/E/A/S/D/X) ──> /cmd_vel/teleop ──┐
Nav2                     ──> /cmd_vel/nav   ──┤
Estop (Space)            ──> /cmd_vel/stop  ──┤
                                              v
                                     twist_mux  (priority: estop > teleop > nav)
                                              |
                                              v
                                     trajectory_controller  (ramping)
                                              |
                                              v
                                     /cmd_vel ──> Gazebo diff-drive plugin
                                                        |
                                                        v
                                                   /odom/wheel
                                                        |
                                              motor_controller (relay)
                                                        |
                                                        v
                                                   /motor/odom ──┐
                                                                  |
Gazebo IMU ──> /imu/raw ──────────────────────────────────> EKF ─┤
                                                                  |
                                                                  v
                                                               /odom
                                                                  |
                                        ┌─────────────────────────┤
                                        v                         v
                                   slam_toolbox               Nav2 stack
                                   (async_slam)               (AMCL+planner)
                                        |
                                        v
                                     /map
```

## Worlds

Two Gazebo worlds are provided in `start_glad_simulation/worlds/`:

- **warehouse.world** — 20×15 m warehouse with walls, shelves, pillars, and a charging station
- **empty.world** — Empty plane

Select with `world:=<path>` launch argument.

## Configuration

Key config files:

- `glad_simulation_urdf/config/glad_urdf.yaml` — Robot dimensions, masses, sensor layout
- `glad_simulation_sensor_filter/config/glad_ekf_params.yaml` — EKF fusion settings
- `glad_simulation_mapping/config/slam_toolbox.yaml` — SLAM parameters
- `glad_simulation_navigation/config/glad_simulation_nav2_params.yaml` — Nav2 stack config
- `glad_simulation_teleop/config/teleop.yaml` — Teleop key timeout, default speeds
- `glad_simulation_twist_mux/config/twist_mux.yaml` — Priority levels and timeout
- `glad_simulation_trajectory_controller/config/trajectory_controller.yaml` — Accel/decel rates

## Build

```bash
colcon build --packages-select \
  start_glad_simulation \
  glad_simulation_urdf \
  glad_simulation_teleop \
  glad_simulation_twist_mux \
  glad_simulation_trajectory_controller \
  glad_simulation_motor_controller \
  glad_simulation_sensor_filter \
  glad_simulation_mapping \
  glad_simulation_navigation
```

Or build all packages in the workspace:

```bash
colcon build
```

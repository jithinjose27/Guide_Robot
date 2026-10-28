# GLAD Mapping (SLAM Toolbox)

Welcome to the **`glad_mapping`** package! This package provides the robot's ability to explore an unknown environment and draw its own map from scratch using **SLAM (Simultaneous Localization and Mapping)**.

---

## How SLAM Works

Imagine walking into a dark, unfamiliar building with a flashlight. Every step you take, you ask:
- *"Where did I just walk from?"* (Odometry tracking)
- *"What walls or obstacles do I see around me?"* (Laser scanning)
- *"Have I been in this hallway before?"* (Loop closure)

SLAM solves the classic "chicken-and-egg" robotics dilemma: you need a map to know where you are, but you need to know where you are to build a map!

### The SLAM Toolbox Pipeline:
```
           LiDAR (/scan)                 Odometry (/odom)
                 │                              │
                 └──────────────┬───────────────┘
                                ▼
                 ┌─────────────────────────────┐
                 │        SLAM Toolbox         │
                 │     (async_slam_toolbox)    │
                 │  - Scan matching            │
                 │  - Pose graph construction  │
                 │  - Ceres Solver optimization│
                 └──────────────┬──────────────┘
                                │
                 ┌──────────────┴──────────────┐
                 ▼                             ▼
        Topic: /map                    TF Transform:
    (nav_msgs/OccupancyGrid)         map ──► odom
```

---

## What Makes SLAM Toolbox Special?

G.L.A.D uses `slam_toolbox` in **asynchronous mapping mode**:
1. **Pose Graph Optimization (Ceres Solver):** Every key scan is saved as a node in an elastic graph connected by mathematical springs (transform constraints).
2. **Loop Closure Detection:** When the robot drives in a circle and returns to a room it visited earlier, it recognizes the laser signature, snaps the graph together, and eliminates accumulated odometry drift across the entire map!
3. **Large Scale Mapping:** Can easily map buildings, campus floors, and warehouses without running out of memory.

---

## Key Configuration Parameters (`config/slam_toolbox.yaml`)

| Parameter | Value | What it means |
|---|---|---|
| `mode` | `mapping` | Active online map construction. |
| `solver_plugin` | `solver_plugins::CeresSolver` | Non-linear least squares optimization engine. |
| `resolution` | `0.05` | Grid resolution ($5\text{ cm}$ per cell). |
| `max_laser_range` | `10.0 m` | Maximum usable distance for laser rays. |
| `min_laser_range` | `0.05 m` | Minimum threshold to discard self-hits. |
| `do_loop_closing` | `true` | Automatically detects and closes loops. |
| `minimum_travel_distance` | `0.05 m` | Adds a new graph node every $5\text{ cm}$ of robot travel. |
| `minimum_travel_heading` | `0.05 rad` | Adds a new graph node every $\sim 2.8^\circ$ of turning. |

---

## How to Build a Map of Your Space

1. **Start the bringup stack in SLAM mode:**
   ```bash
   ros2 launch glad_bringup glad_bringup_launch.py enable_slam:=true
   ```
2. **Drive the robot slowly:**
   - In a separate terminal, open teleop:
     ```bash
     ros2 run glad_teleop teleop_node
     ```
   - Drive at moderate speeds ($0.1 - 0.2\text{ m/s}$).
   - Loop back to previously visited areas to let loop closure snap the walls straight.
3. **Save your completed map:**
   ```bash
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_floor_map'}}"
   ```
   This generates `my_floor_map.pgm` and `my_floor_map.yaml` in your terminal directory.

---

## Building

```bash
colcon build --packages-select glad_mapping
source install/setup.bash
```

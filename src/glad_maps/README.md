# GLAD Maps

Welcome to the **`glad_maps`** package! This package is the robot's geographic memory. It contains pre-built 2D occupancy grid maps that the G.L.A.D mobile robot uses for autonomous localization and navigation.

---

## What is an Occupancy Grid Map?

When a mobile robot navigates through an indoor environment, it needs a digital floor plan to answer two essential questions:
1. **"Where am I right now?"** (Localization via AMCL)
2. **"How do I get to my goal without hitting obstacles?"** (Global Path Planning via NavFn / A*)

In ROS 2, a 2D occupancy grid map represents the world as a 2D top-down image where every pixel corresponds to a small patch of physical space (typically $5\text{ cm} \times 5\text{ cm}$ per cell):
- **White pixels (free space, cost 0):** Areas where the robot can freely drive (hallways, empty floors).
- **Black pixels (obstacles, cost 100):** Physical barriers (walls, pillars, desks, cabinets) detected by LiDAR beams.
- **Grey pixels (unknown space, cost -1):** Unexplored regions outside boundaries or behind closed doors.

Each map consists of two files:
1. **`<map_name>.pgm`** — A standard grayscale Portable Graymap image holding the raw cell occupancy values.
2. **`<map_name>.yaml`** — The ROS 2 metadata descriptor explaining how to convert pixels into real-world coordinates.

---

## Available Maps

| Map Directory | Resolution | Origin [x, y, yaw] | Environment Description |
|---|---|---|---|
| `robotics_floor/` | 0.05 m/cell (5 cm) | `[-0.506, -3.19, 0.0]` | The university Robotics & Automation department floor, featuring the Programming Lab, KUKA industrial arm station, and Automation Lab. |
| `jithin_home/` | 0.05 m/cell (5 cm) | `[-10.0, -10.0, 0.0]` | Indoor domestic room setup used for initial hardware benchmarking and testing. |

---

## Understanding the YAML Metadata

Here is how `robotics_floor/robotics_floor.yaml` is structured:

```yaml
image: robotics_floor.pgm
mode: trinary
resolution: 0.05
origin: [-0.506, -3.19, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- **`image`**: Relative path to the `.pgm` bitmap image.
- **`resolution`**: The physical size of one pixel in meters ($0.05\text{ m} = 5\text{ cm}$).
- **`origin`**: The real-world coordinates $(x, y, yaw)$ of the bottom-left pixel of the map relative to the map frame $(0,0,0)$.
- **`mode`**: `trinary` classifies pixels into strictly free, occupied, or unknown.
- **`occupied_thresh`**: Pixel probabilities above $65\%$ are treated as solid obstacles.
- **`free_thresh`**: Pixel probabilities below $25\%$ are treated as completely clear driving corridors.

---

## How to Create and Save Your Own Map

Want to use G.L.A.D in your own college lab, office, or living room? Here is the exact workflow:

1. **Launch SLAM Mode:**
   ```bash
   ros2 launch glad_bringup glad_bringup_launch.py enable_slam:=true
   ```
2. **Drive the robot slowly:**
   Use the keyboard teleop or joystick to drive around the space until the entire area is mapped in RViz.
3. **Save your new map:**
   ```bash
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_custom_map'}}"
   ```
4. **Copy the map into `glad_maps`:**
   Create a folder `src/glad_maps/my_custom_map/`, place `my_custom_map.yaml` and `my_custom_map.pgm` inside, re-run `colcon build`, and specify `map_yaml` in your launch commands!

---

## Building & Using in Launch Files

This package installs its map directory directly into the ROS 2 workspace share directory:

```bash
colcon build --packages-select glad_maps
source install/setup.bash
```

In any launch file, you can cleanly access the map path without hardcoding usernames:

```python
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

map_file = PathJoinSubstitution([
    FindPackageShare("glad_maps"),
    "robotics_floor",
    "robotics_floor.yaml"
])
```

# GLAD Simulation Navigation

ROS 2 simulation navigation stack for the G.L.A.D robot based on **Nav2**.

## Launch

```
ros2 launch glad_simulation_navigation glad_simulation_nav2_launch.py map:=/path/to/map.yaml
```

The `map` argument is required — pass the full path to a YAML map file.

## Dependencies

- ROS 2 (`rclpy`, `launch_ros`)
- `navigation2` (all Nav2 packages)
- `nav2_msgs`

## Build

```bash
colcon build --packages-select glad_simulation_navigation
source install/setup.bash
```

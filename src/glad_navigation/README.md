# GLAD Navigation (Nav2) 🧭

Welcome to the **`glad_navigation`** package! This package contains the brain of the G.L.A.D mobile robot's autonomous mobility, powered by the industry-standard **ROS 2 Navigation2 (Nav2)** stack.

---

## 🎯 What Does Nav2 Do?

When you tell G.L.A.D to guide a group of students to the Robotics Lab, Nav2 coordinates several specialized subsystems to get the robot there safely:

```
                  ┌───────────────────────────────┐
                  │          Map Server           │
                  │   Serves static floor plan    │
                  └──────────────┬────────────────┘
                                 │
           ┌─────────────────────┴─────────────────────┐
           ▼                                           ▼
┌──────────────────────┐                     ┌───────────────────┐
│         AMCL         │                     │   Global Planner  │
│  Where is the robot? │                     │    (NavFn / A*)   │
│  (Particle filter)   │                     │ Finds global path │
└──────────┬───────────┘                     └─────────┬─────────┘
           │                                           │
           │ odom ──► map                              │ Planned Path
           ▼                                           ▼
┌────────────────────────────────────────────────────────────────┐
│                   Controller Server (Path Follower)            │
│               - Regulated Pure Pursuit Controller              │
│               - Checks local rolling obstacles (Costmap)       │
│               - Produces velocity commands (/cmd_vel/nav)      │
└────────────────────────────────┬───────────────────────────────┘
                                 │
                                 ▼
                     glad_twist_mux (Priority 40)
```

---

## 🧩 The Core Components Explained

### 1. AMCL (Adaptive Monte Carlo Localization)
- **Role:** Keeps track of the robot's pose $(x, y, \theta)$ relative to the static map.
- **How it works:** Spreads a cloud of hundreds of candidate particles across the map. As the robot drives and the LiDAR senses walls, particles that disagree with real-time laser beams are discarded, while matching particles multiply. The cloud quickly collapses into the robot's true physical position!
- **Default Pose:** Initialized automatically to $(0.0, 0.0, \text{yaw}=0.0)$ on startup.

### 2. Costmaps: Global vs. Local
A **costmap** inflates obstacles so the robot's physical body doesn't clip walls or furniture:
- **Global Costmap:** Spans the entire building floor. Uses the static map + real-time laser obstacles to calculate the shortest path.
- **Local Costmap:** A $5\text{ m} \times 5\text{ m}$ rolling window centered around the moving robot. Detects moving people, temporary obstacles, and chairs.
- **Footprint:** Defined as a safe $0.50\text{ m} \times 0.50\text{ m}$ square bounding box:
  ```yaml
  footprint: "[ [0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25] ]"
  ```

### 3. Controller: Regulated Pure Pursuit
- **Role:** Steers the robot along the calculated path.
- **Why Regulated Pure Pursuit?**
  Unlike older controllers that try to pivot erratically, Regulated Pure Pursuit dynamically slows down when approaching sharp corners and automatically regulates linear speed near tight obstacles.
  - Linear cruising speed: `0.10 m/s` (gentle, safe laboratory tour pace).
  - Lookahead distance: `0.45 m` (smooth trajectory following).
  - Reverse disabled: `allow_reversing: false` (robot always moves forward facing its LiDAR).

### 4. Behavior Recovery Server
If someone steps directly in front of G.L.A.D, the robot doesn't freeze forever! It invokes recovery behaviors in order:
1. **Wait:** Pauses briefly to let the person walk past.
2. **BackUp:** Safely backs away from the obstacle.
3. **Spin:** Slowly turns in place to search for an alternate corridor.

---

## 🛠️ Key Topics & Interfaces

| Topic | Type | Role |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Input filtered LiDAR scans. |
| `/odom` | `nav_msgs/Odometry` | Filtered robot odometry from `glad_sensor_filter`. |
| `/map` | `nav_msgs/OccupancyGrid` | The loaded static floor map. |
| `/cmd_vel/nav` | `geometry_msgs/Twist` | Autonomous velocity commands sent to the twist multiplexer. |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Allows RViz 2D Pose Estimate relocation. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Interactive navigation goal from RViz. |

---

## 🚀 Building & Launching

```bash
colcon build --packages-select glad_navigation
source install/setup.bash
```

### Launch Navigation with Default Robotics Floor Map:
```bash
ros2 launch glad_navigation glad_nav2_launch.py
```

### Launch Navigation with Your Own Custom Map:
```bash
ros2 launch glad_navigation glad_nav2_launch.py map:=/path/to/my_map.yaml
```

### Sending a Goal via Python:
You can command G.L.A.D programmatically using the Nav2 Simple Commander API (as demonstrated in `glad_tour`):
```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()
goal = PoseStamped()
goal.header.frame_id = 'map'
goal.pose.position.x = 3.5
goal.pose.position.y = 1.2
goal.pose.orientation.w = 1.0

navigator.goToPose(goal)
```

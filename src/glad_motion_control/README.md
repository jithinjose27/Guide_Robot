# GLAD Motion Control

Welcome to the **`glad_motion_control`** pipeline! This is the motor execution and drive system of G.L.A.D. It turns high-level desires ("drive to coordinates $(3.5, 2.0)$" or "turn left") into physical wheel rotations on the ground.

---

## The Motion Control Pipeline

Safety and smoothness are paramount when controlling physical robots. If you send raw, abrupt step changes in velocity directly to DC motors, wheels slip, gears strip, and the robot jerks violently!

The G.L.A.D motion pipeline solves this with a 4-stage layered architecture:

```
┌────────────────────────┐      ┌────────────────────────┐      ┌────────────────────────┐
│      Teleop Keyboard   │      │     Nav2 Navigation    │      │     Emergency Stop     │
│   (/cmd_vel/teleop)    │      │     (/cmd_vel/nav)     │      │    (/cmd_vel/stop)     │
└───────────┬────────────┘      └───────────┬────────────┘      └───────────┬────────────┘
            │                               │                               │
            └───────────────────────┐       │       ┌───────────────────────┘
                                    ▼       ▼       ▼
                        ┌───────────────────────────────────────┐
                        │        Stage 1: glad_twist_mux        │
                        │  - Priority multiplexer               │
                        │  - Estop(100) > Teleop(70) > Nav(40)  │
                        └───────────────────┬───────────────────┘
                                            │ /cmd_vel/raw
                                            ▼
                        ┌───────────────────────────────────────┐
                        │   Stage 2: glad_trajectory_controller │
                        │  - Slew-rate acceleration limiter     │
                        │  - Smooth ramping (accel / decel)     │
                        └───────────────────┬───────────────────┘
                                            │ /cmd_vel
                                            ▼
                        ┌───────────────────────────────────────┐
                        │     Stage 3: glad_motor_controller    │
                        │  - Differential drive kinematics      │
                        │  - RPM to PWM (-255 to 255)           │
                        └───────────────────┬───────────────────┘
                                            │ UART Serial (PWM_L, PWM_R\n)
                                            ▼
                        ┌───────────────────────────────────────┐
                        │       Stage 4: ESP32 Hardware         │
                        │  - Dual H-Bridge motor drivers        │
                        │  - Optical wheel encoders             │
                        └───────────────────┬───────────────────┘
                                            │ UART Serial (TICKS_L, TICKS_R\n)
                                            ▼
                        ┌───────────────────────────────────────┐
                        │      Encoder Odometry Publisher       │
                        │  - Delta ticks -> distance traveled   │
                        │  - Publishes /motor/odom              │
                        └───────────────────────────────────────┘
```

---

## Subpackages Detailed

### 1. `glad_twist_mux` — Priority Multiplexer
- **Role:** Mediates between human operators and autonomous navigation.
- **Priority Hierarchy:**
  1. **Emergency Stop (`/cmd_vel/stop`, priority 100):** Immediately locks output velocity to 0 when toggled by spacebar or safety node.
  2. **Keyboard Teleop (`/cmd_vel/teleop`, priority 70):** Whenever a human presses a key, teleop instantly takes control over autonomous navigation.
  3. **Nav2 Autonomy (`/cmd_vel/nav`, priority 40):** Autonomous path following runs when teleop is idle.
- **Safety Timeout:** If no message is received on an active channel within `0.5s`, it falls through to lower priorities.

### 2. `glad_trajectory_controller` — Slew-Rate Limiter
- **Role:** Eliminates sudden acceleration spikes.
- **Math Behind Ramping:**
  $$\Delta v = \text{clamp}(v_{\text{target}} - v_{\text{current}}, -a_{\text{max}} \cdot dt, +a_{\text{max}} \cdot dt)$$
  Uses $a_{\text{linear}} = 0.2\text{ m/s}^2$ for smooth takeoff, and $d_{\text{linear}} = 0.4\text{ m/s}^2$ for responsive stopping.

### 3. `glad_motor_controller` — ESP32 Differential Drive Bridge
- **Role:** Communicates with the low-level microcontroller over serial.
- **Kinematic Formulas:**
  - Forward velocity $v_x$ ($\text{m/s}$) and angular rate $\omega_z$ ($\text{rad/s}$) map to wheel velocities:
    $$v_{\text{right}} = v_x + \frac{\omega_z \cdot L}{2}, \quad v_{\text{left}} = v_x - \frac{\omega_z \cdot L}{2}$$
    *(where $L = 0.325\text{ m}$ is the wheel separation distance)*.
  - Encoder Odometry Integration:
    $$d_{\text{center}} = \frac{d_{\text{right}} + d_{\text{left}}}{2}, \quad \Delta\theta = \frac{d_{\text{right}} - d_{\text{left}}}{L}$$
    $$x_{k+1} = x_k + d_{\text{center}} \cos\theta, \quad y_{k+1} = y_k + d_{\text{center}} \sin\theta$$
- **Topic Published:** `/motor/odom` (`nav_msgs/Odometry`).

### 4. `glad_teleop` — Interactive Keyboard Control
- **Controls:**
  ```
         Q=↖  W=↑  E=↗
         A=←  S=■  D=→
              X=↓
  ```
  - `W` / `X`: Forward / Reverse
  - `A` / `D`: Pivot Left / Pivot Right
  - `Q` / `E`: Curve Forward Left / Curve Forward Right
  - `S`: Instant Stop
  - `Spacebar`: Toggle Emergency Stop (E-Stop)
  - `+` / `-`: Increase / Decrease linear speed
  - `[` / `]`: Increase / Decrease angular speed

---

## Building & Launching

```bash
colcon build --packages-select glad_teleop glad_twist_mux glad_trajectory_controller glad_motor_controller
source install/setup.bash
```

### Launch entire motion stack:
```bash
ros2 launch glad_teleop teleop_launch.py
```

### Drive manually in its own dedicated window:
```bash
ros2 run glad_teleop teleop_node
```

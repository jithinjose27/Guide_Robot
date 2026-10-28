# GLAD Tour (Autonomous AI Tour Guide) 🎓🤖

Welcome to the **`glad_tour`** package! This is where robotics navigation and modern Generative AI meet.

G.L.A.D acts as an autonomous docent: it navigates a sequence of designated laboratory stations, stops at each one, and delivers location-specific technical explanations to visitors using Google Gemini and Text-to-Speech (TTS).

---

## 🌟 How the Tour Works

```
                   1. Initialize Nav2 BasicNavigator
                                │
                                ▼
                     2. Set Initial Pose (0, 0)
                                │
                                ▼
             3. Wait for Nav2 to activate (Costmaps ready)
                                │
                                ▼
       ┌──────────────► 4. Send Goal Pose (goToPose) ──────────────┐
       │                                                           │
       │                   5. Monitor Navigation                   │
       │                      (non-blocking loop)                  │
       │                               │                           │
       │                               ▼                           │
       │                    6. Arrived at Station!                 │
       │                               │                           │
       │                               ▼                           │
       │             7. Generate AI Spoken Commentary              │
       │             - Send prompt to Google Gemini 2.5 Flash      │
       │             - Fallback to curated script if offline       │
       │                               │                           │
       │                               ▼                           │
       │                 8. Speak aloud via USB Speaker            │
       │                      (gTTS + mpg123/ffplay)               │
       │                               │                           │
       └───────────────────────────────┴───────────────────────────┘
```

---

## 📍 Designated Tour Stations

| Stop | Location | Coordinates [X, Y, Yaw] | What G.L.A.D Explains |
|---|---|---|---|
| **Station 1** | **Programming Lab** | $x=3.44\text{ m}, y=0.595\text{ m}, \text{yaw}=1.21\text{ rad}$ | Introduces the robotics software engineering lab, algorithm design, and ROS 2 control systems. |
| **Station 2** | **Robotics Lab** | $x=3.17\text{ m}, y=3.66\text{ m}, \text{yaw}=1.35\text{ rad}$ | Highlights the industrial 6-DOF KUKA manipulator, forward/inverse kinematics, and precision pick-and-place. |
| **Station 3** | **Automation Lab** | $x=3.60\text{ m}, y=0.452\text{ m}, \text{yaw}=1.45\text{ rad}$ | Details industrial electro-pneumatic and hydraulic circuits, fluid dynamics, and automated sequential logic. |

---

## 💡 Generative AI with Offline Resilience

To ensure that **anyone can run this project out of the box** without getting stuck on API keys or internet issues:
- **Online with API Key:** If `GEMINI_API_KEY` is exported, G.L.A.D queries **`gemini-2.5-flash-lite`** on the fly. Gemini produces fresh, conversational responses tailored to the lab context.
- **Offline / No Key:** If no API key is provided, G.L.A.D automatically falls back to built-in, professionally written technical commentaries for each station!

---

## 🛠️ How to Customize for Your Own Lab or School

Want to create a tour of your own college or office? Just edit the `waypoints` array in `glad_tour.py`:

```python
waypoints = [
    {
        "name": "Library Entrance",
        "x": 5.2,
        "y": 1.4,
        "z": 0.0,
        "w": 1.0,
        "prompt": "We have arrived at the central library. Welcome the visitors and describe the research archives."
    },
    ...
]
```

### Tips for Finding Coordinates:
1. Open RViz with your saved map.
2. Use the **"Publish Point"** or **"2D Nav Goal"** tool.
3. Look at the terminal echoing the clicked point or run:
   ```bash
   ros2 topic echo /clicked_point
   ```

---

## 🚀 Building & Launching

```bash
colcon build --packages-select glad_tour
source install/setup.bash
```

### Set Your Gemini API Key (Optional):
```bash
export GEMINI_API_KEY="AIzaSy..."
```

### Launch the Tour:
```bash
ros2 launch glad_tour glad_tour_launch.py
```
*(Or launch the complete system including sensors and Nav2 via `ros2 launch glad_final_tour glad_final_tour_launch.py`)*.

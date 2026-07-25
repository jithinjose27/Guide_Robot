# glad_simulation_urdf

G.L.A.D. Simulation URDF Package

## Overview

This package provides the Unified Robot Description Format (URDF) model
and simulation assets for the G.L.A.D. robot. It contains the robot's
kinematic and visual description, along with launch files, RViz
configurations, and simulation parameters.

## Contents

- `urdf/` — Robot URDF and Xacro model files
- `launch/` — Launch files for spawning and visualising the robot
- `config/` — Configuration parameter files
- `rviz/` — RViz display configuration files
- `resource/` — Ament index resource marker

## Usage

```bash
colcon build --packages-select glad_simulation_urdf
source install/setup.bash
```

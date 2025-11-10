# TRCOOL Project

## Overview

**TRCOOL** (Trajectory and Robot Coordination for Optimal Operations and Logistics)  
is a research-oriented robotics project focusing on **autonomous navigation**, **trajectory optimization**, and **robotic coordination**.  
The repository contains modular work packages (WPs) that cover the entire system — from low-level motion planning to ROS-based control and simulation.

---

## Work Package Structure

| Work Package | Title | Description |
|---------------|--------|-------------|
| **WP1** | Path Planning | Algorithms for global route generation, obstacle avoidance, and trajectory smoothing |
| **WP2** | --- | --- |
| **WP3** | --- | --- |
| **WP4** | ROS Integration & Simulation | ROS-based implementation, Gazebo/RViz visualization, and message communication |

---

## WP1 — Path Planning

The **Path Planning module (WP1)** provides the foundation for autonomous navigation.  
It focuses on generating a **collision-free**, **optimized**, and **smooth** trajectory between a start and goal position.

Key features include:
- Implementation of the **A\*** algorithm for grid-based motion planning  
- **Obstacle inflation** using morphological operations to simulate safety margins  
- **B-spline interpolation** for trajectory smoothing  
- **Matplotlib visualization** for debugging and analysis  

📁 Related code and documentation:  
[`/WP1_PathPlanning/README.md`](WP1_PathPlanning/README.md)

## WP4 — ROS Integration


**WP4** focuses on bringing the algorithms developed in WP1–WP3 into a **ROS environment**.
This includes:

- ROS nodes for path planning, map publishing, and robot control

- Interfacing between nav_msgs/Path, geometry_msgs/PoseStamped, etc.

- Visualization in **RViz**

- Simulation in **Gazebo**

📁 Related folder:
[`/WP4_ROS/README.md`](WP4_ROS/README.md)


## 🛠️ Dependencies

Make sure you have the following installed:

```bash
pip install numpy matplotlib scipy
```

For ROS components (WP4):
```bash

```

## 🧭 Repository Structure

```kotlin
TRCOOL_Project/
│
├── Image/
│   └── Path_Trajectory.png
│
├── WP1_PathPlanning/
│   ├── Path_Planning.py
│   ├── README.md
│
├── WP4_ROS/
│   ├── launch/
│   ├── src/
│   ├── README.md
│
└── README.md   ← (this file)

```








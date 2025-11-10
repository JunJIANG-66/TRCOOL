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
| **WP2** | Perception & Mapping *(optional placeholder)* | Environment understanding and grid/occupancy mapping |
| **WP3** | Control & Kinematics *(optional placeholder)* | Low-level motor control and trajectory tracking |
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

Quick example (simplified A* usage):

```python
path = astar(grid, start=(2, 2), goal=(22, 22))
plt.plot(*zip(*path), 'o--')
plt.show()

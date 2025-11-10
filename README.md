## Path Planning with A* and Spline Smoothing

### Overview

This project demonstrates a complete **path planning** pipeline that combines:

1. Map creation with obstacles

2. A* search algorithm for global path planning

3. Obstacle inflation for safety margins

4. Path smoothing using cubic B-spline interpolation

5. Visualization with Matplotlib

The workflow is inspired by autonomous navigation systems in robotics, where a robot must find a collision-free and smooth path from a start point to a goal.

---

### Step 1. Map Creation and Obstacles

- `0` represents free space  
- `1` represents occupied cells (obstacles)


1 represents occupied cells (obstacles)



```python

# Example: Map creation with obstacles

grid = np.zeros((25, 25))
grid[5:10, 5:8] = 1 # obstacle 1
grid[12:17, 10:13] = 1 # obstacle 2
```

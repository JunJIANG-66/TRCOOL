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

A 2D occupancy grid map is generated using NumPy, where:

- `0` represents free space  
- `1` represents occupied cells (obstacles)



```python
# Example: Map creation with obstacles

grid = np.zeros((25, 25))

# Define several rectangular obstacles

grid[5:10, 5:8] = 1     # obstacle 1
grid[12:17, 10:13] = 1  # obstacle 2
grid[7:12, 17:20] = 1   # obstacle 3
grid[15:22, 4:6] = 1    # obstacle 4
```

The **start** and **goal** positions are defined as:

 ```python
start = (2, 2)
goal = (22, 22)
```

This simulates a navigation environment where the robot must move from the lower-left corner to the upper-right, avoiding multiple obstacles.

### Step 2. A* Path Planning (an example)

The A* algorithm is implemented to find the **shortest path** between start and goal:

- Supports **8-directional movement** (diagonal moves allowed)

- Uses **Euclidean distance** as the heuristic

- Maintains open and closed sets using Python’s heapq for efficiency

Key logic snippet:

```python
moves = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]
tentative_g = g_score[current] + np.hypot(dx, dy)
f_score = tentative_g + np.hypot(goal[0]-nx, goal[1]-ny)
heapq.heappush(open_set, (f_score, neighbor))
```

This ensures optimal pathfinding performance even in cluttered maps.

### Step 3. Obstacle Inflation (Safety Distance)

To simulate **robot size** or **safety clearance**, obstacles are expanded using `scipy.ndimage.binary_dilation`.
If no path is found at a certain safety distance, the program automatically reduces it until a valid path appears.

```python
inflated_grid = binary_dilation(grid, iterations=safety_distance).astype(int)
```

This mechanism prevents the robot from planning too close to obstacles and improves robustness in narrow environments.

### Step 4. Path Smoothing (Cubic B-Spline)

After A* generates a discrete grid-based path, it is often **jerky**.
To make it suitable for real robot motion or visualization, a **cubic B-spline interpolation** is used to smooth it.

```python
tck, u = splprep([path[:,0], path[:,1]], s=2)
x_smooth, y_smooth = splev(np.linspace(0, 1, 400), tck)
```
This produces a continuous and differentiable curve that better represents a realistic trajectory.

### Step 5. Visualization
The results are plotted using Matplotlib:

- Gray blocks → obstacles

- Transparent layer → inflated safety zone

- Blue dashed line → A* path

- Red curve → smoothed spline trajectory

- Green point → start

- Purple point → goal

```python
plt.imshow(inflated_grid.T, origin='lower', cmap='Greys', alpha=0.3)
plt.imshow(grid.T, origin='lower', cmap='gray_r', alpha=0.8)
plt.plot(path[:,0], path[:,1], 'o--', label="A* Path (discrete)")
plt.plot(x_smooth, y_smooth, '-', label="Smoothed Trajectory")
```

Final output clearly shows the **safe and smooth route** from start to goal.






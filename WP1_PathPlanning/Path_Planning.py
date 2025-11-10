import numpy as np
import matplotlib.pyplot as plt
import heapq
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation

# =========================================
# Step 1. Create map and multiple obstacles
# =========================================
grid = np.zeros((25, 25))

# Define several obstacles (rectangular blocks)
grid[5:10, 5:8] = 1     # obstacle 1
grid[12:17, 10:13] = 1  # obstacle 2
grid[7:12, 17:20] = 1   # obstacle 3
grid[15:22, 4:6] = 1    # obstacle 4

# Start and goal
start = (2, 2)
goal = (22, 22)

# =========================================
# Step 2. A* Path Planning (with robust indexing)
# =========================================
def astar(grid_map, start, goal):
    rows, cols = grid_map.shape
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    moves = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]  # 8 directions

    visited = set()
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        if current in visited:
            continue
        visited.add(current)

        for dx, dy in moves:
            nx = current[0] + dx
            ny = current[1] + dy
            neighbor = (nx, ny)
            # check bounds
            if not (0 <= nx < rows and 0 <= ny < cols):
                continue
            # check occupancy (use explicit 2D indexing)
            if grid_map[nx, ny] != 0:
                continue
            tentative_g = g_score[current] + np.hypot(dx, dy)
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                # heuristic = euclidean distance
                f_score = tentative_g + np.hypot(goal[0]-nx, goal[1]-ny)
                heapq.heappush(open_set, (f_score, neighbor))
    # no path found
    return []

# =========================================
# Step 3. Inflate obstacles but fallback if no path
# =========================================
initial_safety = 1  # the value you tried that caused error
safety_distance = initial_safety
path = []

# Try decreasing safety_distance until path found or reach 0
while safety_distance >= 0:
    inflated_grid = binary_dilation(grid, iterations=safety_distance).astype(int)
    path = astar(inflated_grid, start, goal)
    if len(path) > 0:
        print(f"Path found with safety_distance = {safety_distance}")
        break
    else:
        print(f"No path with safety_distance = {safety_distance}, decreasing...")
        safety_distance -= 1

if len(path) == 0:
    raise RuntimeError("No valid path found even with safety_distance = 0. Check map or start/goal positions.")

path = np.array(path)  # shape (N,2) now guaranteed because path non-empty

# =========================================
# Step 4. Spline Smoothing (safe checks)
# =========================================
if path.shape[0] > 3:
    # Use cubic B-spline for smooth trajectory  (Cubic Spline Interpolation)
    tck, u = splprep([path[:,0], path[:,1]], s=2)
    unew = np.linspace(0, 1, 400)
    x_smooth, y_smooth = splev(unew, tck)
else:
    # Not enough points for spline: just use the path points
    x_smooth, y_smooth = path[:,0], path[:,1]

# =========================================
# Step 5. Visualization with safety shading
# =========================================
plt.figure(figsize=(8,8))

# Plot inflated safety zone (light gray transparent) using the inflated_grid we used
plt.imshow(inflated_grid.T, origin='lower', cmap='Greys', alpha=0.3)

# Plot original obstacles (dark gray)
plt.imshow(grid.T, origin='lower', cmap='gray_r', alpha=0.8)

# Plot A* discrete path
plt.plot(path[:,0], path[:,1], 'o--', label="A* Path (discrete)", color='tab:blue')

# Plot smoothed spline trajectory
plt.plot(x_smooth, y_smooth, '-', label="Smoothed Trajectory", color='tab:red', linewidth=2)

# Start and goal markers
plt.scatter(*start, color='green', s=80, label='Start')
plt.scatter(*goal, color='purple', s=80, label='Goal')

# Formatting
plt.title(f"A* Path with Smoothed Trajectory and Safety Margin (used={safety_distance})")
plt.axis("equal")
plt.legend()
plt.grid(True)
plt.show()

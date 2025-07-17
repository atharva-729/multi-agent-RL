import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import json

# ==== Sample real agent data ====
agent_paths = {
    "0": [{"x": -0.041, "y": -0.023}, {"x": -0.254, "y": -0.147}, {"x": -0.455, "y": -0.212}, {"x": -0.612, "y": -0.28}, {"x": -0.737, "y": -0.309}, {"x": -0.872, "y": -0.346}],
    "1": [{"x": -0.067, "y": -0.082}, {"x": -0.298, "y": -0.414}, {"x": -0.592, "y": -0.752}, {"x": -0.933, "y": -1.118}, {"x": -1.43, "y": -1.626}],
    "2": [{"x": 0.059, "y": 0.079}, {"x": 0.284, "y": 0.38}, {"x": 0.517, "y": 0.75}, {"x": 0.718, "y": 1.109}, {"x": 0.936, "y": 1.466}]
}

# ==== Define maze layout ====
maze = np.array([
    [0, 1, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [1, 1, 0, 1, 0, 1, 1],
    [0, 0, 0, 0, 0, 0, 0],
    [1, 1, 0, 1, 1, 1, 1],
    [0, 0, 0, 1, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0]
])

# ==== Normalize all agent paths to grid coordinates (0-6) ====
all_x = []
all_y = []
for path in agent_paths.values():
    all_x.extend([p['x'] for p in path])
    all_y.extend([p['y'] for p in path])

min_x, max_x = min(all_x), max(all_x)
min_y, max_y = min(all_y), max(all_y)

def scale(val, min_val, max_val):
    return 6 * (val - min_val) / (max_val - min_val)

scaled_paths = {}
for agent, path in agent_paths.items():
    scaled_path = []
    for p in path:
        x_scaled = scale(p['x'], min_x, max_x)
        y_scaled = scale(p['y'], min_y, max_y)
        scaled_path.append((x_scaled, y_scaled))
    scaled_paths[agent] = scaled_path

# ==== Plot Maze and Agent Paths ====
fig, ax = plt.subplots(figsize=(7, 7))

# Draw maze blocks
for y in range(maze.shape[0]):
    for x in range(maze.shape[1]):
        if maze[y, x] == 1:
            ax.add_patch(patches.Rectangle((x, 6 - y), 1, 1, color='black'))

# Draw grid lines
for i in range(8):
    ax.axhline(i, color='gray', lw=0.5)
    ax.axvline(i, color='gray', lw=0.5)

# Draw agents
colors = ['red', 'blue', 'green']
for i, (agent, path) in enumerate(scaled_paths.items()):
    path = np.array(path)
    ax.plot(path[:, 0], path[:, 1], color=colors[i], linewidth=2, label=f'Agent {agent}')
    ax.plot(path[0, 0], path[0, 1], 'o', color=colors[i], label=f'Start {agent}')
    ax.plot(path[-1, 0], path[-1, 1], 's', color=colors[i], label=f'End {agent}')

# Styling
ax.set_xlim(0, 7)
ax.set_ylim(0, 7)
ax.set_aspect('equal')
ax.invert_yaxis()
ax.legend()
ax.set_title('Agent Paths on Maze Grid')
plt.tight_layout()
plt.show()

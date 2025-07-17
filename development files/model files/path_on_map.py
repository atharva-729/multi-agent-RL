import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn

# Dummy policy for now — replace with your actual model
class DummyPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc = nn.Linear(25, 4)  # 5x5 obs → 4 actions

    def forward(self, obs):
        return self.fc(obs)

    def act(self, obs):
        with torch.no_grad():
            logits = self.forward(obs)
            return torch.argmax(logits).item()

# Define 2D map: 0 = free, 1 = wall
grid = np.array([
    [1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1],
])

H, W = grid.shape

# 5x5 crop centered on agent
def get_local_obs(grid, pos, size=5):
    pad = size // 2
    padded = np.pad(grid, pad_width=pad, mode='constant', constant_values=1)
    x, y = pos
    x += pad
    y += pad
    obs = padded[x - pad:x + pad + 1, y - pad:y + pad + 1]
    return torch.tensor(obs.flatten(), dtype=torch.float32)

# Step function
def step(pos, action, grid):
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right
    dx, dy = moves[action]
    new_x, new_y = pos[0] + dx, pos[1] + dy

    # check bounds and obstacles
    if (0 <= new_x < H) and (0 <= new_y < W) and (grid[new_x, new_y] == 0):
        return (new_x, new_y)
    return pos  # stay in place if move is invalid

# Run simulation
def run_simulation(agent_start, policy, grid, max_steps=50):
    pos = agent_start
    path = [pos]

    for _ in range(max_steps):
        obs = get_local_obs(grid, pos)
        action = policy.act(obs)
        pos = step(pos, action, grid)
        path.append(pos)

        # stop if goal or loop
        if len(path) > 5 and path[-1] == path[-2]:  # stuck
            break

    return path

# Initialize dummy policy
policy = DummyPolicy()

# Run 3 agents
start_positions = [(1, 1), (5, 1), (3, 5)]
paths = [run_simulation(start, policy, grid) for start in start_positions]

# Plotting
fig, ax = plt.subplots()
ax.imshow(grid, cmap='gray_r')

colors = ['b', 'orange', 'green']
for i, path in enumerate(paths):
    xs, ys = zip(*path)
    ax.plot(ys, xs, 'o-', color=colors[i], label=f'Agent {i}')

ax.set_title("Agent Paths on Grid")
ax.set_xlabel("Y")
ax.set_ylabel("X")
ax.legend()
plt.gca().invert_yaxis()
plt.grid(True)
plt.tight_layout()
plt.show()

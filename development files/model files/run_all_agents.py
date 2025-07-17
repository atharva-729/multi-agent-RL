import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# --- Model Definition (same as training one) ---
class SimpleAgentNet(nn.Module):
    def __init__(self):
        super(SimpleAgentNet, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(4, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

    def forward(self, x):
        return self.fc(x)

# --- Agent wrapper ---
def run_agent(agent_id, checkpoint_path, start_pose):
    model = SimpleAgentNet()
    model.load_state_dict(torch.load(checkpoint_path, map_location='cpu'))
    model.eval()

    x, y, theta = start_pose
    stop_flag = 0.0
    path = [(x, y)]

    for _ in range(30):  # max 30 steps
        inp = torch.tensor([[x, y, theta, stop_flag]], dtype=torch.float32)
        with torch.no_grad():
            out = model(inp)

        dx, dy = out[0][0].item(), out[0][1].item()
        x += dx
        y += dy
        theta += 0.0  # assume no heading change for now
        stop_flag = (dx**2 + dy**2) ** 0.5  # crude stop metric

        path.append((x, y))
        if stop_flag < 0.05:
            break

    return path

# --- Start poses from image ---
start_poses = {
    0: (11, 3, 0.0),   # blue
    1: (10, 7, 0.0),   # orange
    2: (13, 8, 0.0),   # green
}

# --- Run all agents ---
paths = {}
for agent_id in [0, 1, 2]:
    path = run_agent(
        agent_id,
        f'agent_{agent_id}_final_mappo.pth',
        start_poses[agent_id]
    )
    paths[agent_id] = path

# --- Plotting ---
colors = {0: 'blue', 1: 'orange', 2: 'green'}
map_img = mpimg.imread('b24c26ae-7927-4a53-ba10-99cc7afda7d9.jpg')

fig, ax = plt.subplots()
ax.imshow(map_img)

for agent_id, path in paths.items():
    xs, ys = zip(*path)
    ax.plot(ys, xs, marker='o', label=f'Agent {agent_id}', color=colors[agent_id])

ax.legend()
plt.title("Paths of All 3 Agents")
plt.show()

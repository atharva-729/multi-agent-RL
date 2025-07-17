import torch
import torch.nn as nn

# Dumb environment input: zeros
INPUT_VECTOR = torch.zeros(638)  # Size must match trained input size

class Actor(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim),
        )
    def forward(self, x):
        return self.fc(x)

def load_agent_model(path, input_dim=638, output_dim=4):
    model = Actor(input_dim, output_dim)
    model.load_state_dict(torch.load(path, map_location='cpu'))
    model.eval()
    return model

def get_path(agent_model, agent_name):
    print(f"\nPath for {agent_name}:")
    x, y, theta = 0.0, 0.0, 0.0
    path = [(x, y)]
    for step in range(20):  # max 20 steps
        obs = torch.zeros(638)  # dummy input
        out = agent_model(obs)
        dx, dy, dtheta, stop_flag = out.tolist()
        x += dx
        y += dy
        theta += dtheta
        path.append((round(x, 2), round(y, 2)))
        print(f"Step {step+1}: x={x:.2f}, y={y:.2f}, theta={theta:.2f}, stop_flag={stop_flag:.2f}")
        if abs(stop_flag) > 1.0:
            break
    return path

agent_paths = {
    "Agent 0": "agent_0_final_mappo.pth",
    "Agent 1": "agent_1_final_mappo.pth",
    "Agent 2": "agent_2_final_mappo.pth",
}

for name, file in agent_paths.items():
    model = load_agent_model(file)
    get_path(model, name)

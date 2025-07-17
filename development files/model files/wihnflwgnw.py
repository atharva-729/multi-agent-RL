import torch
import torch.nn as nn
import numpy as np

class SimpleAgentNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(638, 128),
            nn.Tanh(),
            nn.Linear(128, 4)
        )

    def forward(self, x):
        return self.fc(x)

# Load model
model = SimpleAgentNet()
state_dict = torch.load("agent_0_final_mappo.pth", map_location=torch.device('cpu'))
model.load_state_dict(state_dict)
model.eval()

# Simulate agent path
pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
path = [pose.copy()]

for step in range(10):
    # Dummy observation (you'll replace this with real data later)
    obs = torch.randn(1, 638)

    # Get action
    with torch.no_grad():
        action = model(obs).numpy().squeeze()

    dx, dy, dtheta, stop_flag = action

    # Update position
    pose[0] += dx
    pose[1] += dy
    pose[2] += dtheta
    path.append(pose.copy())

    print(f"Step {step+1}: x={pose[0]:.2f}, y={pose[1]:.2f}, theta={pose[2]:.2f}, stop_flag={stop_flag:.2f}")

    if stop_flag > 0.9:  # you can adjust this threshold
        print("Stopping as per stop_flag")
        break

# Final path
print("\nFull path:")
for i, p in enumerate(path):
    print(f"{i}: ({p[0]:.2f}, {p[1]:.2f}, θ={p[2]:.2f})")

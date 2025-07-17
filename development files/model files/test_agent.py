import torch
import torch.nn as nn

class SimpleAgentNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(638, 128),   # Input layer: 638 → 128
            nn.Tanh(),
            nn.Linear(128, 4)      # Output layer: 128 → 4
        )

    def forward(self, x):
        return self.fc(x)

# Load the model
model = SimpleAgentNet()
state_dict = torch.load("agent_0_final_mappo.pth", map_location=torch.device('cpu'))
model.load_state_dict(state_dict)
model.eval()

# Test dummy input
dummy_input = torch.randn(1, 638)  # A fake 638-dim input
output = model(dummy_input)

print("Output:", output)

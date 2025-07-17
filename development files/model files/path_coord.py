import torch

# Replace with the path to your .pth file
pth_file = "agent_0_final_mappo.pth"

# Load the file on CPU
model_data = torch.load(pth_file, map_location=torch.device('cpu'))

# Print the structure of the file
if isinstance(model_data, dict):
    print("\nTop-level keys:")
    for key in model_data.keys():
        print("  -", key)

    # If there's a state_dict, print its keys
    if 'state_dict' in model_data:
        print("\nKeys in state_dict:")
        for key in model_data['state_dict'].keys():
            print("  -", key)
else:
    print("Model is not a dict. Type:", type(model_data))

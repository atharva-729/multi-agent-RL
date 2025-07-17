import serial
import time

# Update COM port to match your HC-05
bt = serial.Serial('COM8', 9600)  # Replace 'COM8' with your actual port
time.sleep(2)

print("Bluetooth motor controller ready.")

# Command options
commands = {
    'F': 'Forward Both',
    'B': 'Backward Both',
    'S': 'Stop Both',
    'L': 'Forward Left',
    'l': 'Backward Left',
    'x': 'Stop Left',
    'R': 'Forward Right',
    'r': 'Backward Right',
    'y': 'Stop Right'
}

try:
    while True:
        print("\nAvailable commands:")
        for k, v in commands.items():
            print(f"{k} - {v}")
        cmd = input("Enter command: ").strip()

        if cmd in commands:
            bt.write(cmd.encode())
            print(f"Sent: {commands[cmd]}")
        else:
            print("Invalid command")

except KeyboardInterrupt:
    print("Exiting...")
    bt.close()

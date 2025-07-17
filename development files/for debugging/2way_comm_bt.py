import serial
import time

bt = serial.Serial('COM8', 9600)  # Replace 'COM8' with your Bluetooth serial port
time.sleep(2)  # Give some time for Arduino to reset if needed

print("Sending message to Arduino via Bluetooth...")
bt.write(b'hello from Python\n')

# Wait for response
response = bt.readline().decode().strip()
print("Arduino replied:", response)

# Optional: send more data in loop
for i in range(5):
    msg = f"Ping {i}\n"
    bt.write(msg.encode())
    print(f"Sent: {msg.strip()}")
    resp = bt.readline().decode().strip()
    print(f"Received: {resp}")
    time.sleep(1)

bt.close()

import serial
import time

bt = serial.Serial('COM14', 9600)  # Replace with your actual HC-05 port
print("Listening for data from Arduino via Bluetooth...")

try:
    while True:
        if bt.in_waiting:
            data = bt.readline().decode().strip()
            print(f"Received: {data}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting")
    bt.close()

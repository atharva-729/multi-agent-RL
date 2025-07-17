import serial
import time

# Connect to HC-05
try:
    bt = serial.Serial('COM9', 9600)  # replace COMx with actual port
    print("your bluetooth device is connected")
    # List of coordinates to send
    coords = [(600, 0)]

    for x, y in coords:
        # Send next coordinate
        msg = f"{x},{y}\n"
        bt.write(msg.encode())
        print(f"Sent: {msg.strip()}")

        # Wait for "Target reached!"
        while True:
            line = bt.readline().decode().strip()
            print(f"Received: {line}")
            if "Target reached!" in line:
                print("Robot is ready")
                break

except serial.SerialException as e:
    print(f"Serial port error: {e}")
    
except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
    bt.close()
finally:
    print("lol")
import serial
import time

# Replace with your actual Bluetooth COM port
bluetooth_port = 'COM9'
baud_rate = 9600

try:
    bt = serial.Serial(bluetooth_port, baud_rate)
    time.sleep(2)  # Wait for connection to stabilize

    # Step 1: Send coordinates
    coords = input("Enter coordinates (x,y) to send to Arduino: ").strip()
    bt.write((coords + '\n').encode('utf-8'))
    print(f"Sent: {coords}")
    print("\n--- Listening for data from Arduino ---")

    # Step 2: Listen to Serial1 debug output
    while True:
        if bt.in_waiting:
            data = bt.readline().decode().strip()
            print(f"Received: {data}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    bt.close()
    print("Bluetooth connection closed.")
except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    if 'bt' in locals() and bt.is_open:
        bt.close()
        print("Bluetooth connection closed.")

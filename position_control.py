import serial
import time

# --- CONFIG ---
PORT = 'COM8'  # <-- Change this to your HC-05 COM port
BAUD = 9600
GAINS = {
    'KP': 1.2,
    'KI': 0.05,
    'KD': 0.01
}

# --- MAIN ---
try:
    print(f"Connecting to {PORT}...")
    bt = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Allow time for Arduino to reset if needed

    # Send each gain command
    for name, value in GAINS.items():
        command = f"{name} {value}\n"
        bt.write(command.encode('utf-8'))
        print(f">> Sent: {command.strip()}")
        time.sleep(0.1)  # slight delay between commands

    print("\n--- Waiting for Arduino Responses ---")
    start = time.time()
    while time.time() - start < 5:  # Read for 5 seconds
        if bt.in_waiting:
            response = bt.readline().decode('utf-8', errors='ignore').strip()
            if response:
                print(f"<< {response}")

except serial.SerialException as e:
    print(f"Serial error: {e}")
finally:
    if 'bt' in locals() and bt.is_open:
        bt.close()
        print("Serial port closed.")

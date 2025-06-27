import serial

# Replace with your COM port. On Linux use something like '/dev/rfcomm0'
port = 'COM8'  # Change this!
baud = 9600

try:
    bt = serial.Serial(port, baud, timeout=1)
    print(f"Connected to {port} at {baud} baud.\n")

    while True:
        line = bt.readline().decode('utf-8').strip()
        if line:
            print(f"> {line}")

except serial.SerialException as e:
    print("Could not open serial port:", e)

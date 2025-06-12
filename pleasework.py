import serial
import time

ser = serial.Serial('COM8', 9600)
try:
    while True:
        ser.write(b'F')
        time.sleep(1)
        ser.write(b'B')
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
    ser.close()

import serial
ser = serial.Serial('COM8', 9600)
ser.write(b'hello from bt\n')

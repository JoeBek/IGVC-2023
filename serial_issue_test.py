import serial

ser = serial.Serial('COM3')
if ser.is_open:
    print("Serial port is open")
else:
    print("Serial port is not open")

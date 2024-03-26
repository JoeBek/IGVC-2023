import serial

ser = serial.Serial('COM5')
ser = serial.Serial('COM5')
if ser.is_open:
    print("Serial port is open")
else:
    print("Serial port is not open")

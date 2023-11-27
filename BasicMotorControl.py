# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
SerialObj = serial.Serial('COM4') # COMxx  format on Windows
                  # ttyUSBx format on Linux
SerialObj.baudrate = 115200  # set Baud rate to 9600
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1
time.sleep(1)

for i in range(5):
    # Go in the direction of small wheels at 100 speed
    SerialObj.write(b'!G 1 -100\r')
    SerialObj.write(b'!G 2 100\r')
    time.sleep(1)

SerialObj.write(b'!G 1 0\r')
SerialObj.write(b'!G 2 0\r')


time.sleep(1)
SerialObj.close()      # Close the port

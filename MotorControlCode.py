# Python code transmits a byte to Arduino /Microcontroller
import serial
import time

MotorSerialObj = serial.Serial('COM5') # COMxx  format on Windows
                  # ttyUSBx format on Linux
MotorSerialObj.baudrate = 115200  # set Baud rate to 9600
MotorSerialObj.bytesize = 8   # Number of data bits = 8
MotorSerialObj.parity  ='N'   # No parity
MotorSerialObj.stopbits = 1   # Number of Stop bits = 1

BluetoothSerialObj = serial.Serial('COM3') # COMxx  format on Windows
                  # ttyUSBx format on Linux
BluetoothSerialObj.baudrate = 9600  # set Baud rate to 9600
BluetoothSerialObj.bytesize = 8   # Number of data bits = 8
BluetoothSerialObj.parity  ='N'   # No parity
BluetoothSerialObj.stopbits = 1   # Number of Stop bits = 1

initialTime = time.time_ns()

ONE_SECOND_DELAY = 1000000000

currentKey = "none"

while True:

    if (BluetoothSerialObj.inWaiting() > 0):
        BluetoothString = BluetoothSerialObj.readline()
    else:
        BluetoothString = ""

    if BluetoothString == b"RIGHT PRESSED\n":
        currentKey = "right"
    elif BluetoothString == b"RIGHT RELEASED\n":
        if (currentKey == "right"):
            currentKey = "none"
    elif BluetoothString == b"LEFT PRESSED\n":
        currentKey = "left"
    elif BluetoothString == b"LEFT RELEASED\n":
        if (currentKey == "left"):
            currentKey = "none"
    elif BluetoothString == b"UP PRESSED\n":
        currentKey = "up"
    elif BluetoothString == b"UP RELEASED\n":
        if (currentKey == "up"):
            currentKey = "none"
    elif BluetoothString == b"DOWN PRESSED\n":
        currentKey = "down"
    elif BluetoothString == b"DOWN RELEASED\n":
        if (currentKey == "down"):
            currentKey = "none"
    elif BluetoothString == b"END PROGRAM\n":
        print("Program Ended")
        currentKey = "none"
        MotorSerialObj.write(b'!G 1 0\r')
        MotorSerialObj.write(b'!G 2 0\r')
        break

    if (time.time_ns() > initialTime + (ONE_SECOND_DELAY*0.1)):
        initialTime = time.time_ns()
        if (currentKey == "up"):
            print("Going Forward")
            MotorSerialObj.write(b'!G 1 -200\r')
            MotorSerialObj.write(b'!G 2 200\r')
        elif (currentKey == "down"):
            print("Going Backwards")
            MotorSerialObj.write(b'!G 1 200\r')
            MotorSerialObj.write(b'!G 2 -200\r')
        elif (currentKey == "left"):
            print("Going Left")
            MotorSerialObj.write(b'!G 1 -100\r')
            MotorSerialObj.write(b'!G 2 300\r')
        elif (currentKey == "right"):
            print("Going Right")
            MotorSerialObj.write(b'!G 1 -300\r')
            MotorSerialObj.write(b'!G 2 100\r')
        elif (currentKey == "none"):
            print("Stopping")
            MotorSerialObj.write(b'!G 1 0\r')
            MotorSerialObj.write(b'!G 2 0\r')

MotorSerialObj.close()      # Close the port
BluetoothSerialObj.close()      # Close the port

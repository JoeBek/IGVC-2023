# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
from pynput import keyboard
from pynput.keyboard import Key


SerialObj = serial.Serial('COM5') # COMxx  format on Windows
                  # ttyUSBx format on Linux
SerialObj.baudrate = 9600  # set Baud rate to 9600
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1

def on_key_release(key):
    if key == Key.right:
        print("Right key released")
        SerialObj.write(b"RIGHT RELEASED\n")
    elif key == Key.left:
        print("Left key released")
        SerialObj.write(b"LEFT RELEASED\n")
    elif key == Key.up:
        print("Up key released")
        SerialObj.write(b"UP RELEASED\n")
    elif key == Key.down:
        print("Down key released")
        SerialObj.write(b"DOWN RELEASED\n")
    elif key == Key.space:
        print("Activating blinking light")
        SerialObj.write(b"SPACE RELEASED\n")
    elif key == Key.esc:
        SerialObj.write(b"END PROGRAM\n")
        print("Stopping Motor")
        time.sleep(1)
        SerialObj.close()      # Close the port
        exit()
    else:
        SerialObj.write(b"Q\n")
        print("Stopping Motor")

def on_key_press(key):
    if key == Key.right:
        print("Right key clicked")
        SerialObj.write(b"RIGHT PRESSED\n")
    elif key == Key.left:
        print("Left key clicked")
        SerialObj.write(b"LEFT PRESSED\n")
    elif key == Key.up:
        print("Up key clicked")
        SerialObj.write(b"UP PRESSED\n")
    elif key == Key.down:
        print("Down key clicked")
        SerialObj.write(b"DOWN PRESSED\n")
    elif key == Key.esc:
        SerialObj.write(b"END PROGRAM\n")
        print("Stopping Motor")
        time.sleep(1)
        SerialObj.close()      # Close the port
        exit()

with keyboard.Listener(
        on_press=on_key_press,
        on_release=on_key_release) as listener:
    listener.join()

#listener = keyboard.Listener(
#    on_press=on_key_press,
#    on_release=on_key_release)
#listener.start()

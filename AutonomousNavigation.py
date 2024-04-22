# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
import pyzed.sl as sl
import math
import numpy as np
import cv2
import os
from MotorControlAPI import MotorController
from Control import Control
from GPS_API import GPS

# Importing pygame module
import pygame
from pygame.locals import *


#Defining Important Constants
DEPTH_VALUE = 300 # when objects are closer than DEPTH_VALUE, the robot will turn to avoid them
MAX_DEPTH = 400
ROBOT_SPEED = 20
ONE_SECOND_DELAY = 1000000000
ROBOT_WIDTH = 100 #Horizontal width of robot in cm

WAYPOINT_LATITUDE = 37.228889
WAYPOINT_LONGITUDE = -80.4155556

def parseManualControls(bluetoothString, currentDirection):
    if bluetoothString == b"RIGHT PRESSED\n":
        return "right"
    elif bluetoothString == b"RIGHT RELEASED\n":
        if (currentDirection == "right"):
            return "stop"
    elif bluetoothString == b"LEFT PRESSED\n":
        return "left"
    elif bluetoothString == b"LEFT RELEASED\n":
        if (currentDirection == "left"):
            return "stop"
    elif bluetoothString == b"UP PRESSED\n":
        return "up"
    elif bluetoothString == b"UP RELEASED\n":
        if (currentDirection == "up"):
            return "stop"
    elif bluetoothString == b"DOWN PRESSED\n":
        return "down"
    elif bluetoothString == b"DOWN RELEASED\n":
        if (currentDirection == "down"):
            return "stop"
    elif bluetoothString == b"SPACE RELEASED\n":
        return "autonomous"
    elif bluetoothString == b"END PROGRAM\n":
        return "shutdown"
    else:
        return currentDirection
    
def sendMotorCommand(motorObj, command, lastCommandTime, leftSpeed=0, rightSpeed=0):
    if (time.time_ns() > lastCommandTime + (ONE_SECOND_DELAY*0.1)):
            lastCommandTime = time.time_ns()
            if (command == "up"):
                print("Going Forward")
                motorObj.forward(ROBOT_SPEED)
            elif (command == "down"):
                print("Going Backwards")
                motorObj.backward(ROBOT_SPEED)
            elif (command == "left"):
                print("Going Left")
                motorObj.turnLeft(ROBOT_SPEED*2)
            elif (command == "right"):
                print("Going Right")
                motorObj.turnRight(ROBOT_SPEED*2)
            elif (command == "custom"): #Defaults to stopping if not left and right speed are given
                print("Custom speed")
                motorObj.customMovement(leftSpeed, rightSpeed)
            elif (command == "stop"):
                print("Stopping")
                motorObj.stop()
    return lastCommandTime

#--------------------------------------------------------------
# Little guy reminding you it will be ok
#--------------------------------------------------------------
'''
        , < it will be ok >
     0
    -|- 
     /\


'''
#----------------------------------------------------------------------
# Setting up usb connections
#----------------------------------------------------------------------
motors = MotorController('COM5')

BluetoothSerialObj = serial.Serial('COM4') # COMxx  format on Windows
                  # ttyUSBx format on Linux
BluetoothSerialObj.baudrate = 9600  # set Baud rate to 9600
BluetoothSerialObj.bytesize = 8   # Number of data bits = 8
BluetoothSerialObj.parity  ='N'   # No parity
BluetoothSerialObj.stopbits = 1   # Number of Stop bits = 1

# Setting up Zed camera for both line detection and object detection
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.CENTIMETER
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
error = zed.open(init_params)

#----------------------------------------------------------------------
# Setting up overhead view
#----------------------------------------------------------------------

# initiate pygame and give permission
# to use pygame's functionality.
pygame.init()
# create the display surface object
# of specific dimension.
window = pygame.display.set_mode((600, 600))
pygame.display.set_caption('Overhead View')
# Fill the screen with white color
window.fill((255, 255, 255))
# Draws the surface object to the screen.
pygame.display.update()

#----------------------------------------------------------------------
# Setup of important variables
#----------------------------------------------------------------------

#Control unit handles all pathing logic through zed display

control = Control(zed)

# Holds the time the last motor command was sent
# Used to ensure that motor commands are not sent more than 10 times per second
timeOfLastCommand = time.time_ns()

# Holds the command about to be sent
currentCommand = "stop"

# Boolean to keep track of whether the robot is in autonomous mode, starts in manual control mode
doAutonomous = False

#----------------------------------------------------------------------
# Main loop
#----------------------------------------------------------------------
while True:

    # Check for a bluetooth command
    if (BluetoothSerialObj.inWaiting() > 0):
        BluetoothString = BluetoothSerialObj.readline()
    else:
        BluetoothString = ""

    #TODO: Add watchdog timer

    # Always parse the bluetooth command even if the robot is in autonomous mode to look out for an stop command
    manualControlCommand = parseManualControls(BluetoothString, currentCommand)

    if (manualControlCommand == "shutdown"):
        print("Program Ended")
        motors.shutDown()
        break
    elif (manualControlCommand == "autonomous"):
        BluetoothSerialObj.write(b"Z\n")
        doAutonomous = not doAutonomous
    elif (~doAutonomous): #Only set the currentKey to the manual command if the robot is not in autonomous mode
        currentCommand = manualControlCommand
    else:
        currentCommand = "stop"


    window.fill((255, 255, 255))
    pygame.draw.circle(window, (0,0,0), Control.coordinateTransform(0, 0), 20) #Draw a black circle representing the robot in the overhead view
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(0, 0), Control.coordinateTransform(0, 600), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(100, 0), Control.coordinateTransform(100, 600), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(200, 0), Control.coordinateTransform(200, 600), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(-100, 0), Control.coordinateTransform(-100, 600), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(-200, 0), Control.coordinateTransform(-200, 600), width=2)

    #Add horizontal grid lines
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(-300, 100), Control.coordinateTransform(300, 100), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(-300, 200), Control.coordinateTransform(300, 200), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(-300, 300), Control.coordinateTransform(300, 300), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(-300, 400), Control.coordinateTransform(300, 400), width=2)
    pygame.draw.line(window, (0,0,0), Control.coordinateTransform(-300, 500), Control.coordinateTransform(300, 500), width=2)


    # if the robot is in autonomous mode, set the current command to the best course of action according to the autonomous control logic
    if (doAutonomous):
        leftSpeed, rightSpeed = control.runAutonomousControls(zed)
        timeOfLastCommand = sendMotorCommand(motors, "custom", timeOfLastCommand, leftSpeed, rightSpeed)
    else:
        # send the next course of action to the motors based on manual controls
        timeOfLastCommand = sendMotorCommand(motors, currentCommand, timeOfLastCommand)

    pygame.display.update() #Update the overhead view


    #TODO: add logic to check if the robot is near the destination waypoint
    #       if the robot is near the waypoint, shut down
        
        

    # This is required, I don't know why but opencv doesn't work without this if statement contained within the main loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Shut down everything and close open ports
motors.shutDown()
BluetoothSerialObj.close()      # Close the port

"""
===================================================================================================================
                                                INSTRUCTIONS
===================================================================================================================

To utilize this API, put the line "from MotorControlAPI import MotorController" at the top of your file.

To instantiate a motor controller object, use the command:
    myVar = MotorController('COMx')
where 'COMx' is replaced with the COM port of the motor controller.

This class has the following functions for moving the robot:
    myVar.forward(speed)
    myVar.backward(speed)
    myVar.turnLeft(speed)
    myVar.turnRight(speed)
for these functions, the input variable "speed" is an integer with a range of 0-100 where 100 is full speed and 0 is stopped.
Additionally, "forward" is defined as the direction of the small wheels on the robot.

The class has 2 functions for stopping the robot:
    myVar.stop()
    myVar.shutDown()

The stop() function is the primary way to get the robot to stop moving. The shutDown() command closes the
    serial port for the motor controller and should only be used as an emergency stop command or as the
    last command of your script.

===================================================================================================================
"""
import serial
import time

class MotorController:
    def __init__(self, comPort):
        # Setting up serial communication with the motor controller
        self.MotorSerialObj = serial.Serial(comPort) 
        self.MotorSerialObj.baudrate = 115200  # set Baud rate to 115200
        self.MotorSerialObj.bytesize = 8   # Number of data bits = 8
        self.MotorSerialObj.parity  ='N'   # No parity
        self.MotorSerialObj.stopbits = 1   # Number of Stop bits = 1
        self.MotorSerialObj.write(b'!MG\r')

    #Tells both motors to go forward (the direction of the small wheels)
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def forward(self, speed):
        if (speed < 0
            ):
            exit()

        print("forward")
        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        motorSpeedValue1 = int(-10*speed) # Left motor
        motorSpeedValue2 = int(10*speed)  # Right motor
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Tells both motors to go backwards (the direction of the big wheels)
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def backward(self, speed):
        if (speed < 0):
            exit()
        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        motorSpeedValue1 = int(10*speed)
        motorSpeedValue2 = int(-10*speed)
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Turns the robot to the left, resulting in the right wheel turning and the left wheel stationary
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def turnLeft(self, speed):
        if (speed < 0):
            exit()
        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        #motorSpeedValue1 = int(5*speed)
        motorSpeedValue1 = 0
        motorSpeedValue2 = int(10*speed)
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Turns the robot to the right, resulting in the left wheel turning and the right wheel stationary
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def turnRight(self, speed):
        if (speed < 0):
            exit()
        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        motorSpeedValue1 = int(-10*speed)
        #motorSpeedValue2 = int(-5*speed)
        motorSpeedValue2 = 0
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Sets the speed of the robot's wheels to a custom value specified by right and left speed
    # the speed values represent a percentage of maximum speed and can be negative
    # IMPORTANT NOTE: turning the left and right wheel in opposite directions causes the robot to be stuck in place
    def customMovement(self, right_speed, left_speed):
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        motorSpeedValue1 = int(-10*left_speed) # Left motor
        motorSpeedValue2 = int(10*right_speed)  # Right motor
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Tells motors to stop
    def stop(self):
        self.MotorSerialObj.write(b'!G 1 0\r')
        self.MotorSerialObj.write(b'!G 2 0\r')

    #Emergency shuts down motors and closes the serial port
    def shutDown(self):
        self.MotorSerialObj.write(b'!EX\r')
        time.sleep(1)
        self.MotorSerialObj.close()      # Close the port

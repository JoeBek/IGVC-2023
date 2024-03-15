# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
import pyzed.sl as sl
import math
from MotorControlAPI import MotorController
from GPS_API import GPS

#Defining Important Constants
DEPTH_VALUE = 300 # when objects are closer than DEPTH_VALUE, the robot will turn to avoid them
MAX_DEPTH = 400
ROBOT_SPEED = 20
ONE_SECOND_DELAY = 1000000000
ROBOT_WIDTH = 100 #Horizontal width of robot in cm


def identifyCourseOfAction(obstacleList):
    return obstacleList

#This function currently only works for a single obstacle at a time
# TODO: update this with logic to handle multiple objects
def findObstacle(depthList):
    obstacleIndexList = []
    for i in range(len(depthList)):
        if (depthList[i] < DEPTH_VALUE):
            obstacleIndexList.append(i)
    
    return obstacleIndexList

def identifyExtraneousObjects(obstacleLocationList, leftImage, depthList):
    obstacleCenter = (obstacleLocationList[0] + obstacleLocationList[-1])/2

    HALF_FOV_DEGREES = 55
    CENTER_PIXEL = (leftImage.get_width() / 2)
    
    #Assuming obstacle on right
    # PROPER LOGIC ---------------
    # 1. Find angle theta from midline to closest point of object
    #       theta = 55*(pixelValue - centerPixelValue)/(half of length of image) (Gives units of degrees)
    # 2. Find horizontal distance to closest point of object
    #       M = depthValue * sin(theta)
    # 3. Compare horizontal distance to width of robot
    #       if (M < 0.5 meters) -> avoid obstacle, otherwise continue forward

    # Logic for ignoring obstacles too far left or right
    if (obstacleCenter > (leftImage.get_width() / 2)):  # if the obstacle is on the right
        #Find theta
        leftMostPixelOfObstacleOnRight = None
        for i in range(len(obstacleLocationList)):
            if (obstacleLocationList[i] > (leftImage.get_width() / 2)):
                leftMostPixelOfObstacleOnRight = i
                break
        assert(leftMostPixelOfObstacleOnRight != None)  # TODO: otherwise, obstacle was not even on the right side to begin with

        if (leftMostPixelOfObstacleOnRight):
            theta_degrees = 1.000 * HALF_FOV_DEGREES * (leftMostPixelOfObstacleOnRight - CENTER_PIXEL)/CENTER_PIXEL #Angle to object in degrees
            theta_radians =  theta_degrees * math.pi/180 # Angle to object in radians

            #Find distance to center
            depthToObstacle = depthList[leftMostPixelOfObstacleOnRight]
            horizDistToObjectFromCenterpoint = depthToObstacle * math.sin(theta_radians)

            #Compare horizontal distance to width of robot
            if (horizDistToObjectFromCenterpoint < (ROBOT_WIDTH/2)):
                #Turn left to avoid obstacle
                return "left"
        
    if (obstacleCenter <= (leftImage.get_width() / 2)):  # if the obstacle is on the left
        #Find theta
        rightMostPixelOfObstacleOnLeft = None
        for i in range(len(obstacleLocationList), 0, -1):
            if (obstacleLocationList[i] <= (leftImage.get_width() / 2)):
                rightMostPixelOfObstacleOnLeft = i
                break

        assert(rightMostPixelOfObstacleOnLeft != None)  # TODO: otherwise, obstacle was not even on the left side to begin with

        if (rightMostPixelOfObstacleOnLeft):
            theta_degrees = 1.000 * HALF_FOV_DEGREES * (CENTER_PIXEL - rightMostPixelOfObstacleOnLeft)/CENTER_PIXEL #Angle to object in degrees
            theta_radians =  theta_degrees * math.pi/180 # Angle to object in radians

            #Find distance to center
            depthToObstacle = depthList[rightMostPixelOfObstacleOnLeft]
            horizDistToObjectFromCenterpoint = depthToObstacle * math.sin(theta_radians)

            #Compare horizontal distance to width of robot
            if (horizDistToObjectFromCenterpoint < (ROBOT_WIDTH/2)):
                #Turn right to avoid obstacle
                return "right"
        
        return "up"


def runDepthSensor(zed):
    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    error = zed.grab(runtime_params)
    zed.retrieve_image(leftImage, sl.VIEW.LEFT)
    zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image
    centerY = int(leftImage.get_height() / 2)
    depthList = []  # stores depth values of pixels on center horizontal line of image
    for currentX in range(leftImage.get_width()):
        error, currentDepth = leftDepthMatrix.get_value(currentX, centerY)
        # cleans up data
        if (currentDepth > MAX_DEPTH):
            currentDepth = MAX_DEPTH
        elif math.isnan(currentDepth):
            currentDepth = MAX_DEPTH
        elif (currentDepth < 0):
            currentDepth = 0
        depthList.append(currentDepth)
    
    obstacleLocationList = findObstacle(depthList)

    if (len(obstacleLocationList) == 0):
        #print("No obstacle")
        return "up"
    

    retVal = identifyExtraneousObjects(obstacleLocationList, leftImage, depthList)

    return retVal



def runAutonomousControls(zed):
    depthSensorDirection = runDepthSensor(zed) # Finds the best course of action according to the depth sensor

    #TODO: Identify road lines
    #TODO: Implement logic to avoid obstacles while not crossing road lines

    return depthSensorDirection


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
    
def sendMotorCommand(motorObj, command, lastCommandTime):
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
            elif (command == "stop"):
                print("Stopping")
                motorObj.stop()
    return lastCommandTime

#----------------------------------------------------------------------
# Setting up usb connections
#----------------------------------------------------------------------
motors = MotorController('COM4')
gps = GPS("COM7", 40, -80)

BluetoothSerialObj = serial.Serial('COM3') # COMxx  format on Windows
                  # ttyUSBx format on Linux
BluetoothSerialObj.baudrate = 9600  # set Baud rate to 9600
BluetoothSerialObj.bytesize = 8   # Number of data bits = 8
BluetoothSerialObj.parity  ='N'   # No parity
BluetoothSerialObj.stopbits = 1   # Number of Stop bits = 1

zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.CENTIMETER
error = zed.open(init_params)

#----------------------------------------------------------------------
# Setup of important variables
#----------------------------------------------------------------------

initialTime = time.time_ns()

currentCommand = "stop"

doAutonomous = False

#----------------------------------------------------------------------
# Main loop
#----------------------------------------------------------------------
while True:

    if (BluetoothSerialObj.inWaiting() > 0):
        BluetoothString = BluetoothSerialObj.readline()
    else:
        BluetoothString = ""

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

    if (doAutonomous):
        currentCommand = runAutonomousControls(zed)

    initialTime = sendMotorCommand(motors, currentCommand, initialTime)
            
motors.shutDown()
BluetoothSerialObj.close()      # Close the port

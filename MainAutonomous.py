"""
Module for controlling robot autonomously based on object and line detection.

Authors: Harrison, Shreya, James
Date: March 23, 2024
"""

from enum import Enum
from MotorControlAPI import MotorController
from GPS_API import GPS
import serial
import pyzed.sl as sl
import time
import math


# important constants
OBSTACLE_AVOIDANCE_DEPTH_CM = 300  # when this close (in cm) to obstacle, the robot will turn to avoid them
LINE_AVOIDANCE_DEPTH_CM = 300  # depth at which robot decides to avoid line (in cm)
MAX_DETECTION_DEPTH_CM = 400
ROBOT_SPEED = 20
NANOSEC_PER_SEC = 1000000000  # used for setting delay between motor control sec
ROBOT_WIDTH = 100  # horizontal width of robot (in cm)

# TODO: will not work yet (b/c it doesn't disregard extraneous objects first before choosing closest object)
MULTIPLE_OBSTACLES_ATTEMPT = False  # whether to use multiple obstacle detection or not

class Direction(Enum):
    """
    Directions robot can head toward using motor controls. Values used for user-friendly output display.
    """
    FORWARD = "forward"
    LEFT = "left"
    RIGHT = "right"
    BACKWARD = "backward"


# ======================================================================================================================
# Line Detection
# ======================================================================================================================

# TODO: implement avoidLine()
def avoidLine():
    """
    Decides what direction the robot should go based on any line detected.

    :return: direction to move toward (as a Direction enum), distance from line in centimeters (as a float).
    """
    sampleDirectionToMove = Direction.FORWARD
    sampleDistanceFromLine_cm = 1000  # in centimeters
    return sampleDirectionToMove, sampleDistanceFromLine_cm


# ======================================================================================================================
# Object Detection
# ======================================================================================================================

def findCoordinatesOfSingleCloseObstacle(depth_list):
    """
    Gets x-coordinates of a close obstacle. Assumes there is only one obstacle.

    :param depth_list: list of depth values corresponding to a horizontal line of pixels

    :return: list of x-coordinates of close obstacle
    """
    obstacleIndexList = []
    for i in range(len(depth_list)):
        if depth_list[i] < OBSTACLE_AVOIDANCE_DEPTH_CM:
            obstacleIndexList.append(i)

    return obstacleIndexList


# TODO: attempt for detecting multiple objects
def findCoordinatesOfMultipleCloseObstacles(depth_list):
    """
    Gets x-coordinates of close obstacles. Pixels corresponding to different objects are separated into different lists.

    :param depth_list: list of depth values corresponding to a horizontal line of pixels

    :return: list of lists of x-coordinates of close obstacles (where each list entry corresponds to different objects)
    """
    obstacleDetected = False
    listOfListsOfObstacleCoordinates = []
    listOfCurrentObstacleCoordinates = []

    for depthListIndex in range(len(depth_list)):
        currentDepth = depth_list[depthListIndex]

        if obstacleDetected:
            if currentDepth > OBSTACLE_AVOIDANCE_DEPTH_CM:
                assert len(listOfCurrentObstacleCoordinates) > 0  # TODO: delete later
                listOfListsOfObstacleCoordinates.append(listOfCurrentObstacleCoordinates)
                listOfCurrentObstacleCoordinates = []
                obstacleDetected = False
            else:  # currentDepth <= MIN_DEPTH_TO_OBSTACLE
                listOfCurrentObstacleCoordinates.append(depthListIndex)
        else:  # not obstacleDetected
            if currentDepth <= OBSTACLE_AVOIDANCE_DEPTH_CM:
                assert len(listOfCurrentObstacleCoordinates) == 0  # TODO: delete later
                listOfCurrentObstacleCoordinates.append(depthListIndex)
                obstacleDetected = True

    # adds last remaining obstacle, if any
    if obstacleDetected:
        assert len(listOfCurrentObstacleCoordinates) > 0  # TODO: delete later
        listOfListsOfObstacleCoordinates.append(listOfCurrentObstacleCoordinates)

    return listOfListsOfObstacleCoordinates


def getIndexOfClosestObstacle(obstacle_locations_lists, depth_list):
    """
    Assumes that each list entry is nonempty.
    Location means x-coordinate (index) in depth_list.

    :param obstacle_locations_lists:
    :return:
    """

    if len(obstacle_locations_lists) == 0:
        return -1

    # initialization
    indexOfClosestObject = 0  # index within obstacle_locations_lists
    closestObject = obstacle_locations_lists[indexOfClosestObject]
    indexOfClosestLocationOfClosestObject = 0  # index within list entry of obstacle_location_lists
    closestLocationOfClosestObject = closestObject[indexOfClosestLocationOfClosestObject]
    closestDistanceFromClosestObject = depth_list[closestLocationOfClosestObject]

    # iterates by object
    for indexOfCurrentObject in range(len(obstacle_locations_lists)):
        currentObject = obstacle_locations_lists[indexOfCurrentObject]
        indexOfClosestLocationOfCurrentObject = 0
        closestLocationOfCurrentObject = currentObject[indexOfClosestLocationOfCurrentObject]
        closestDistanceFromCurrentObject = depth_list[closestLocationOfCurrentObject]

        # iterates by location for current object
        for indexOfCurrentLocationOfCurrentObject in range(len(currentObject)):
            currentLocationOfCurrentObject = currentObject[indexOfCurrentLocationOfCurrentObject]
            currentDistanceFromCurrentObject = depth_list[currentLocationOfCurrentObject]

            # compares distances within object
            if currentDistanceFromCurrentObject < closestDistanceFromCurrentObject:
                indexOfClosestLocationOfCurrentObject = indexOfCurrentLocationOfCurrentObject
                closestLocationOfCurrentObject = currentObject[indexOfClosestLocationOfCurrentObject]
                closestDistanceFromCurrentObject = depth_list[closestLocationOfCurrentObject]

        # compares distances among objects
        if closestDistanceFromCurrentObject < closestDistanceFromClosestObject:
            indexOfClosestObject = indexOfCurrentObject
            closestObject = obstacle_locations_lists[indexOfClosestObject]
            indexOfClosestLocationOfClosestObject = indexOfClosestLocationOfCurrentObject
            closestLocationOfClosestObject = closestObject[indexOfClosestLocationOfClosestObject]
            closestDistanceFromClosestObject = depth_list[closestLocationOfClosestObject]

    return indexOfClosestObject


def accountForExtraneousObjects(obstacle_location_list, left_image, depth_list):
    """
    Disregards objects that are too far to the left or right of image. Field of view of camera is very wide, so it
    may pick up on objects that the robot is not head toward.
    
    :param obstacle_location_list:
    :param left_image:
    :param depth_list:

    :return: direction in which to move (as a Direction enum), distance at pixel closest to center pixel (as a float), x-coordinate corresponding to distance
    """
    obstacleCenter = (obstacle_location_list[0] + obstacle_location_list[-1]) / 2

    HALF_FOV_DEGREES = 55
    CENTER_PIXEL = left_image.get_width() / 2

    # Assuming obstacle on right
    # PROPER LOGIC ---------------
    # 1. Find angle theta from midline to closest point of object
    #       theta = 55*(pixelValue - centerPixelValue)/(half of length of image) (Gives units of degrees)
    # 2. Find horizontal distance to closest point of object
    #       M = depthValue * sin(theta)
    # 3. Compare horizontal distance to width of robot
    #       if (M < 0.5 meters) -> avoid obstacle, otherwise continue forward

    # Logic for ignoring obstacles too far left or right
    if obstacleCenter > (left_image.get_width() / 2):  # if the obstacle is on the right
        # Find theta
        leftMostPixelOfObstacleOnRight = None
        for i in range(len(obstacle_location_list)):
            if obstacle_location_list[i] > (left_image.get_width() / 2):
                leftMostPixelOfObstacleOnRight = i
                break
        assert leftMostPixelOfObstacleOnRight is not None  # TODO: otherwise, obstacle was not even on the right side to begin with

        if leftMostPixelOfObstacleOnRight is not None:
            theta_degrees = 1.0 * HALF_FOV_DEGREES * (leftMostPixelOfObstacleOnRight - CENTER_PIXEL) / CENTER_PIXEL  # Angle to object in degrees
            theta_radians = theta_degrees * math.pi / 180  # Angle to object in radians

            # Find distance to center
            depthToObstacle = depth_list[leftMostPixelOfObstacleOnRight]
            horizDistToObjectFromCenterpoint = depthToObstacle * math.sin(theta_radians)

            # Compare horizontal distance to width of robot
            if horizDistToObjectFromCenterpoint < (ROBOT_WIDTH / 2):
                # Turn left to avoid obstacle
                return Direction.LEFT, depthToObstacle, leftMostPixelOfObstacleOnRight
    else:  # if the obstacle is on the left
        # Find theta
        rightMostPixelOfObstacleOnLeft = None
        for i in range(len(obstacle_location_list) - 1, -1, -1):
            if obstacle_location_list[i] <= (left_image.get_width() / 2):
                rightMostPixelOfObstacleOnLeft = i
                break
        assert rightMostPixelOfObstacleOnLeft is not None  # TODO: otherwise, obstacle was not even on the left side to begin with

        if rightMostPixelOfObstacleOnLeft is not None:
            theta_degrees = 1.0 * HALF_FOV_DEGREES * (CENTER_PIXEL - rightMostPixelOfObstacleOnLeft) / CENTER_PIXEL  # Angle to object in degrees
            theta_radians = theta_degrees * math.pi / 180  # Angle to object in radians

            # Find distance to center
            depthToObstacle = depth_list[rightMostPixelOfObstacleOnLeft]
            horizDistToObjectFromCenterpoint = depthToObstacle * math.sin(theta_radians)

            # Compare horizontal distance to width of robot
            if horizDistToObjectFromCenterpoint < (ROBOT_WIDTH / 2):
                # Turn right to avoid obstacle
                return Direction.RIGHT, depthToObstacle, rightMostPixelOfObstacleOnLeft

    return Direction.FORWARD, depth_list[int(obstacleCenter)], int(obstacleCenter)


def avoidObstacle(zed):
    """
    Decides what direction the robot should go based on any obstacle detected.

    :return: direction to move toward (as a Direction enum), distance from obstacle in centimeters (as a float).
    """
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
        if currentDepth > MAX_DETECTION_DEPTH_CM:
            currentDepth = MAX_DETECTION_DEPTH_CM
        elif math.isnan(currentDepth):
            currentDepth = MAX_DETECTION_DEPTH_CM
        elif currentDepth < 0:
            currentDepth = 0
        depthList.append(currentDepth)

    obstacleLocationList = None
    if MULTIPLE_OBSTACLES_ATTEMPT:
        listOfObstacleLocationLists = findCoordinatesOfMultipleCloseObstacles(depthList)
        indexOfClosestObstacle = getIndexOfClosestObstacle(listOfObstacleLocationLists, depthList)
        if indexOfClosestObstacle < 0 or indexOfClosestObstacle >= len(listOfObstacleLocationLists):
            obstacleLocationList = []
        else:
            rawObstacleLocationList = listOfObstacleLocationLists[indexOfClosestObstacle]
            arr = []
            for elem in rawObstacleLocationList:
                arr.append(depthList[elem])
            obstacleLocationList = findCoordinatesOfSingleCloseObstacle(arr)
    else:
        obstacleLocationList = findCoordinatesOfSingleCloseObstacle(depthList)

    if len(obstacleLocationList) == 0:
        # print("No obstacle")
        directionAwayFromObstacle = Direction.FORWARD
        distanceFromObstacle = MAX_DETECTION_DEPTH_CM
        xCoordOfObstacle = leftImage.get_width() / 2
    else:
        directionAwayFromObstacle, distanceFromObstacle, xCoordOfObstacle = accountForExtraneousObjects(obstacleLocationList, leftImage, depthList)

    print("Direction chosen: {0}, Obstacle distance at ({1}, {2}): {3}".format(directionAwayFromObstacle.value, xCoordOfObstacle, centerY, distanceFromObstacle))
    return directionAwayFromObstacle, distanceFromObstacle


# ======================================================================================================================
# Autonomous Detection
# ======================================================================================================================

def avoidLineAndObstacle(zed):
    """
    Decides which direction robot should move based on combined information from line and object detection.

    :return: direction to move toward (as a Direction enum)
    """
    directionAwayFromLine, distanceFromLine = avoidLine()  # direction to move away from line
    directionAwayFromObstacle, distanceFromObstacle = avoidObstacle(zed)  # direction to move away from obstacle

    directionToMove = Direction.FORWARD
    # agreement on which direction to move toward
    if directionAwayFromLine == directionAwayFromObstacle:
        directionToMove = directionAwayFromLine
    # only one of line or obstacle to avoid
    elif directionAwayFromLine == Direction.FORWARD:
        directionToMove = directionAwayFromObstacle
    elif directionAwayFromObstacle == Direction.FORWARD:
        directionToMove = directionAwayFromLine
    # disagreement on which direction to move toward
    elif ((directionAwayFromLine == Direction.LEFT and directionAwayFromObstacle == Direction.RIGHT) or
          (directionAwayFromLine == Direction.RIGHT and directionAwayFromObstacle == Direction.LEFT)):
        if distanceFromLine < LINE_AVOIDANCE_DEPTH_CM:  # move away from line if too close (in cm), no matter what
            directionToMove = directionAwayFromLine
        elif distanceFromObstacle < distanceFromLine:
            directionToMove = directionAwayFromObstacle
        else:  # distanceFromLine <= distanceFromObstacle
            directionToMove = directionAwayFromLine

    print("Direction to move: {0},\n\tLine: ({1}, {2}),\n\tObstacle: ({3}, {4})".format(directionToMove.value,
        directionAwayFromLine.value, distanceFromLine, directionAwayFromObstacle.value, distanceFromObstacle))
    return directionToMove


def runAutonomousControls(zed):
    """

    :param zed:
    :return: command to send to motor controls as a String
    """

    # TODO: Identify road lines
    # TODO: Implement logic to avoid obstacles while not crossing road lines

    depthSensorDirection = avoidLineAndObstacle(zed)  # finds the best course of action according to the depth sensor

    # send appropriate motor control command
    if depthSensorDirection == Direction.FORWARD:
        return "up"
    elif depthSensorDirection == Direction.LEFT:
        return "left"
    elif depthSensorDirection == Direction.RIGHT:
        return "right"
    elif depthSensorDirection == Direction.BACKWARD:
        return "down"
    else:  # should never reach this point (based on current Direction enum class)
        return "stop"


# ======================================================================================================================
# Manual Controls
# ======================================================================================================================

def parseManualControls(bluetooth_string, current_direction):
    if bluetooth_string == b"RIGHT PRESSED\n":
        return "right"
    elif bluetooth_string == b"RIGHT RELEASED\n":
        if current_direction == "right":
            return "stop"
    elif bluetooth_string == b"LEFT PRESSED\n":
        return "left"
    elif bluetooth_string == b"LEFT RELEASED\n":
        if current_direction == "left":
            return "stop"
    elif bluetooth_string == b"UP PRESSED\n":
        return "up"
    elif bluetooth_string == b"UP RELEASED\n":
        if current_direction == "up":
            return "stop"
    elif bluetooth_string == b"DOWN PRESSED\n":
        return "down"
    elif bluetooth_string == b"DOWN RELEASED\n":
        if current_direction == "down":
            return "stop"
    elif bluetooth_string == b"SPACE RELEASED\n":
        return "autonomous"
    elif bluetooth_string == b"END PROGRAM\n":
        return "shutdown"


def sendMotorCommand(motor_obj, command, last_command_time):
    if time.time_ns() > last_command_time + (NANOSEC_PER_SEC * 0.1):
        last_command_time = time.time_ns()
        if command == "up":
            print("Going Forward")
            motor_obj.forward(ROBOT_SPEED)
        elif command == "down":
            print("Going Backwards")
            motor_obj.backward(ROBOT_SPEED)
        elif command == "left":
            print("Going Left")
            motor_obj.turnLeft(ROBOT_SPEED * 2)
        elif command == "right":
            print("Going Right")
            motor_obj.turnRight(ROBOT_SPEED * 2)
        elif command == "stop":
            print("Stopping")
            motor_obj.stop()
    return last_command_time


# ======================================================================================================================
# Main function
# ======================================================================================================================

def cameraSoloTest():
    """
    testing multiple objects

    :return:
    """
    # initialization of stereo camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.CENTIMETER
    error = zed.open(init_params)  # opens the camera
    if error != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera. Error code:", error)
        initializationFailure = True

    while True:
        avoidObstacle(zed)


if __name__ == "__main__":

    # cameraSoloTest()  # TODO: delete later

    # initialization of usb connections
    motors = MotorController('COM4')
    gps = GPS("COM7", 40, -80)
    bluetoothSerialObj = serial.Serial('COM3')  # COMxx format on Windows, ttyUSBx format on Linux
    bluetoothSerialObj.baudrate = 9600  # set Baud rate to 9600
    bluetoothSerialObj.bytesize = 8  # Number of data bits = 8
    bluetoothSerialObj.parity = 'N'  # No parity
    bluetoothSerialObj.stopbits = 1  # Number of Stop bits = 1

    # initialization of stereo camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.CENTIMETER
    error = zed.open(init_params)  # opens the camera
    if error != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera. Error code:", error)
        exit(1)

    # initialization of important variables
    initialTime = time.time_ns()
    currentCommand = "stop"
    doAutonomous = False

    while True:
        if bluetoothSerialObj.inWaiting() > 0:
            BluetoothString = bluetoothSerialObj.readline()
        else:
            BluetoothString = ""

        manualControlCommand = parseManualControls(BluetoothString, currentCommand)
        if manualControlCommand == "shutdown":
            print("Program Ended")
            motors.shutDown()
            break
        elif manualControlCommand == "autonomous":
            bluetoothSerialObj.write(b"Z\n")
            doAutonomous = not doAutonomous
        elif not doAutonomous:  # Only set the currentKey to the manual command if the robot is not in autonomous mode
            currentCommand = manualControlCommand
        else:
            currentCommand = "stop"

        if doAutonomous:
            currentCommand = runAutonomousControls(zed)

        initialTime = sendMotorCommand(motors, currentCommand, initialTime)

    # cleanup
    motors.shutDown()
    bluetoothSerialObj.close()      # closes the port
    zed.close()

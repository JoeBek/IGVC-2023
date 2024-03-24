# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
import pyzed.sl as sl
import math
from MotorControlAPI import MotorController
from GPS_API import GPS


# defining important constants
MIN_DEPTH_TO_OBSTACLE = 300  # distance (in cm) at which robot attempts to avoid obstacle by turning
MAX_DEPTH_FOR_DETECTION = 400  # maximum distance (in cm) used for object detection
ROBOT_SPEED = 20  # speed for moving and turning to use for motor controller
ROBOT_WIDTH = 100  # horizontal width of robot (in cm)


# This function currently only works for a single obstacle at a time
# TODO: update logic to handle multiple obstacles
def getCloseObstacleIndices(depth_list):
    """
    Gets x-coordinates of close obstacle. Since we are only focused on qualification, this function assumes there is
    only one obstacle.

    :param depth_list: list of depth values corresponding to a horizontal line of pixels

    :return: list of x-coordinates of close obstacle
    """
    obstacleIndexList = []
    for i in range(len(depth_list)):
        if depth_list[i] < MIN_DEPTH_TO_OBSTACLE:
            obstacleIndexList.append(i)

    return obstacleIndexList


def identifyExtraneousObjects(obstacle_location_list, left_image, depth_list):
    """

    :param obstacle_location_list:
    :param left_image:
    :param depth_list:
    :return:
    """
    obstacleCenter = (obstacle_location_list[0] + obstacle_location_list[-1]) / 2

    HALF_FOV_DEGREES = 55
    CENTER_PIXEL_OF_IMAGE = (left_image.get_width() / 2)
    
    # Assuming obstacle on right
    # PROPER LOGIC ---------------
    # 1. Find angle theta from midline to closest point of object
    #       theta = 55*(pixelValue - centerPixelValue)/(half of length of image) (Gives units of degrees)
    # 2. Find horizontal distance to closest point of object
    #       M = depthValue * sin(theta)
    # 3. Compare horizontal distance to width of robot
    #       if (M < 0.5 meters) -> avoid obstacle, otherwise continue forward

    # checks if obstacle is too far to right
    if obstacleCenter > CENTER_PIXEL_OF_IMAGE:  # if the obstacle is on the right
        # finds theta (angle from center to object)
        leftPixelOfObject = None  # stores x-coordinate of leftmost pixel on right half of image
        for i in range(len(obstacle_location_list)):
            if obstacle_location_list[i] > CENTER_PIXEL_OF_IMAGE:
                leftPixelOfObject = i
                break

        if leftPixelOfObject is not None:
            theta_degrees = (1.0 * HALF_FOV_DEGREES * (leftPixelOfObject - CENTER_PIXEL_OF_IMAGE) /
                CENTER_PIXEL_OF_IMAGE)
            theta_radians = theta_degrees * math.pi / 180.0

            # finds distance to center
            depthToObstacle = depth_list[leftPixelOfObject]
            horizDistToObjectFromCenterpoint = depthToObstacle * math.sin(theta_radians)

            # compares horizontal distance to width of robot
            if horizDistToObjectFromCenterpoint < (ROBOT_WIDTH/2):
                # Turn left to avoid obstacle
                return "left"

    # checks if obstacle is too far to left
    else:  # if the obstacle is on the left
        # finds theta (angle from center to object)
        rightPixelOfObject = None  # stores x-coordinate of rightmost pixel on left half of image
        for i in range(len(obstacle_location_list), 0, -1):
            if obstacle_location_list[i] <= (left_image.get_width() / 2):
                rightPixelOfObject = i
                break

        if rightPixelOfObject is not None:
            theta_degrees = (1.0 * HALF_FOV_DEGREES * (CENTER_PIXEL_OF_IMAGE - rightPixelOfObject) /
                CENTER_PIXEL_OF_IMAGE)
            theta_radians = theta_degrees * math.pi / 180.0

            # finds distance to center
            depthToObstacle = depth_list[rightPixelOfObject]
            horizDistToObjectFromCenterpoint = depthToObstacle * math.sin(theta_radians)

            # compares horizontal distance to width of robot
            if horizDistToObjectFromCenterpoint < (ROBOT_WIDTH / 2):
                # Turn right to avoid obstacle
                return "right"
        
    return "up"


def runDepthSensor(zed):
    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    imageGrabError = zed.grab(runtime_params)
    if imageGrabError == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(leftImage, sl.VIEW.LEFT)
        zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image
        centerY = int(leftImage.get_height() / 2)
        depthList = []  # stores depth values of pixels on center horizontal line of image
        for currentX in range(leftImage.get_width()):
            currentDepth = leftDepthMatrix.get_value(currentX, centerY)[1]
            # cleans up data
            if currentDepth > MAX_DEPTH_FOR_DETECTION:
                currentDepth = MAX_DEPTH_FOR_DETECTION
            elif math.isnan(currentDepth):
                currentDepth = MAX_DEPTH_FOR_DETECTION
            elif currentDepth < 0:
                currentDepth = 0
            depthList.append(currentDepth)

        obstacleLocationList = getCloseObstacleIndices(depthList)
        if len(obstacleLocationList) == 0:
            # print("No obstacle")
            return "up"

        return identifyExtraneousObjects(obstacleLocationList, leftImage, depthList)
    else:
        return None


def runAutonomousControls(zed):
    depthSensorDirection = runDepthSensor(zed)  # Finds the best course of action according to the depth sensor

    # TODO: Identify road lines
    # TODO: Implement logic to avoid obstacles while not crossing road lines

    return depthSensorDirection


# TODO: use enum class for manual motor commands
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
    NS_PER_SEC = 1000000000
    COMMAND_SEND_RATE_SEC = 0.1  # amount of time (in sec) to wait before sending another command

    # sends commands periodically
    if time.time_ns() > last_command_time + (NS_PER_SEC * COMMAND_SEND_RATE_SEC):
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


if __name__ == "__main__":
    # setup for usb connections
    motors = MotorController('COM4')
    gps = GPS("COM7", 40, -80)

    # setup for bluetooth
    bluetoothSerialObj = serial.Serial('COM3')  # COMxx format on Windows, ttyUSBx format on Linux
    bluetoothSerialObj.baudrate = 9600  # set Baud rate to 9600
    bluetoothSerialObj.bytesize = 8  # Number of data bits = 8
    bluetoothSerialObj.parity = 'N'  # No parity
    bluetoothSerialObj.stopbits = 1  # Number of Stop bits = 1

    # setup for zed camera (for obstacle detection)
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.CENTIMETER
    error = zed.open(init_params)
    if error != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera. Error code:", error)
        exit(1)

    # initialization of other variables
    initialTime = time.time_ns()
    currentCommand = "stop"
    doAutonomous = False

    # main loop
    while True:
        if bluetoothSerialObj.inWaiting() > 0:
            BluetoothString = bluetoothSerialObj.readline()
        else:
            BluetoothString = ""

        # manual controls
        manualControlCommand = parseManualControls(BluetoothString, currentCommand)
        if manualControlCommand == "shutdown":
            print("Program ended")
            motors.shutDown()
            break
        elif manualControlCommand == "autonomous":
            bluetoothSerialObj.write(b"Z\n")
            doAutonomous = not doAutonomous
        elif not doAutonomous:  # sets the currentKey to the manual command only if the robot is not in autonomous mode
            currentCommand = manualControlCommand
        else:
            currentCommand = "stop"

        # manual
        if doAutonomous is not None:
            currentCommand = runAutonomousControls(zed)

        initialTime = sendMotorCommand(motors, currentCommand, initialTime)

    # cleanup
    motors.shutDown()
    gps.closeGPS()
    bluetoothSerialObj.close()      # closes the port
    zed.close()

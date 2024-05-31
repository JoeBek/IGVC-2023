"""
Obstacle avoidance tests using depth sensor.

Download Python ZED SDK by following this tutorial: https://www.stereolabs.com/docs/app-development/python/install

Author: Harrison Bui
Date: February 14, 2024
    Updated: March 12, 2024 (incorporated multiprocessing for using Motor Controls)
"""

from MotorControlAPI import MotorController
import pyzed.sl as sl
import math
import multiprocessing
import sys
import time
import os
import queue

# how close robot should be allowed to approach obstacle (in cm); for captureImages...() functions
MIN_THRESHOLD_DISTANCE_CM = 300
MAX_THRESHOLD_DISTANCE_CM = 325

# lower and upper bounds on depth values (in cm) to consider for obstacle detection; for captureImages...() functions
MIN_OBSTACLE_DEPTH_CM = 0
MAX_OBSTACLE_DEPTH_CM = 400

# speed settings for motor controls
FORWARD_SPEED = 20
TURNING_SPEED = 10

# how often to send the same move/turn command to robot (in ms between commands) to make it do the same thing overtime
COMMAND_SEND_PERIOD_MS = 100

# miscellaneous
PROGRAM_START_TIME_MS = None  # reference start time of tests (in ms) after initialization
MS_PER_SEC = 1000  # number of milliseconds per second

# ======================================================================================================================
# Utility Functions
# ======================================================================================================================


def clipData(data_to_fix, lower_bound, upper_bound):
    """
    Determines new value for given value so that it stays within given bounds. Values outside of bounds get reassigned
    to the closest bound value. Function is used to clean up noise in depth sensor data.

    :param data_to_fix: float value to recompute from
    :param lower_bound: the lowest value acceptable for value
    :param upper_bound: the highest value acceptable for value

    :return: the updated data value as a float
    """
    if data_to_fix > upper_bound or math.isnan(data_to_fix):
        return upper_bound
    elif data_to_fix < lower_bound:
        return lower_bound
    return data_to_fix


def getCoordinatesOfCloseObstacle(image, depth_matrix):
    """
    Detects approximate point on center line of image for any obstacle that may be close. Limited to only detecting a
    single obstacle.

    :param image: image retrieved using ZED SDK
    :param depth_matrix: depth measurement retrieved using ZED SDK (in cm)

    :return: center of obstacle (x, y) as a 2-tuple, or None if no obstacle detected
    """
    # initialization
    centerY = int(image.get_height() / 2)
    depthList = []  # stores depth values of pixels on center horizontal line of image

    # extracts depth values on center horizontal line of image
    for currentX in range(image.get_width()):
        error, currentDepth = depth_matrix.get_value(currentX, centerY)
        # cleans up data
        currentDepth = clipData(currentDepth, MIN_OBSTACLE_DEPTH_CM, MAX_OBSTACLE_DEPTH_CM)
        depthList.append(currentDepth)

    # finds leftmost pixel of obstacle
    leftBoundX = 0
    while leftBoundX < len(depthList) and depthList[leftBoundX] >= MAX_OBSTACLE_DEPTH_CM:
        leftBoundX += 1

    # checks if no obstacle detected
    if leftBoundX >= len(depthList):
        return None

    # finds rightmost pixel of obstacle
    rightBoundX = leftBoundX + 1
    while rightBoundX < len(depthList) and depthList[rightBoundX] < MAX_OBSTACLE_DEPTH_CM:
        rightBoundX += 1

    # gets center pixel between the two boundary pixels
    centerOfObstacleX = int((leftBoundX + rightBoundX) / 2)
    return centerOfObstacleX, centerY


def captureImageAndCheckForObstacle(zed, left_image, left_depth_matrix, runtime_params):
    """
    Captures a single image and detects location and depth value of any close obstacle. Helper function for the
    captureImagesUntil* functions.

    :param zed: the ZED camera whose depth sensor to use
    :param left_image: the Mat object for storing image
    :param left_depth_matrix: the Mat object for storing depth matrix
    :param runtime_params: runtime parameters for ZED camera

    :return: depth value, x, y
    """
    # grabs an image
    error = zed.grab(runtime_params)
    x = int(left_image.get_width() / 2)
    y = int(left_image.get_height() / 2)
    depthValue = None
    if error == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(left_image, sl.VIEW.LEFT)  # gets left image
        zed.retrieve_measure(left_depth_matrix, sl.MEASURE.DEPTH)  # gets left depth image

        # gets depth value of obstacle, if any
        obstacleCoordinates = getCoordinatesOfCloseObstacle(left_image, left_depth_matrix)
        if obstacleCoordinates is None:
            x = int(left_image.get_width() / 2)
            y = int(left_image.get_height() / 2)
        else:
            x, y = obstacleCoordinates
        err, depthValue = left_depth_matrix.get_value(x, y)
        depthValue = clipData(depthValue, MIN_OBSTACLE_DEPTH_CM, MAX_OBSTACLE_DEPTH_CM)
        timestampMillisecond = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()
        print("Time: {0} ms, Distance from camera at ({1}, {2}): {3} cm".format(timestampMillisecond -
            PROGRAM_START_TIME_MS, x, y, depthValue))
    else:
        print("Failed to grab image. Error: {0}".format(error))

    return depthValue, x, y


def captureImagesUntilCloseToObstacle(zed):
    """
    Uses depth sensor to wait until a close obstacle is detected. Used to avoid colliding into obstacles.

    :param zed: the ZED camera whose depth sensor to use

    :return: data of frame in which close obstacle detected as a 3-tuple (x, y, image)
    """
    # initialization for using sensor data
    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtimeParameters = sl.RuntimeParameters()

    # keeps capturing depth images until obstacle detect
    depthValue = None
    x = int(leftImage.get_width() / 2)
    y = int(leftImage.get_height() / 2)
    while depthValue is None or depthValue > MIN_THRESHOLD_DISTANCE_CM:
        depthValue, x, y = captureImageAndCheckForObstacle(zed, leftImage, leftDepthMatrix, runtimeParameters)

    return x, y, leftImage


def captureImagesUntilClear(zed):
    """
    Uses depth sensor to wait until no close obstacle is detected. Used to find open area to move to.

    :param zed: the ZED camera whose depth sensor to use
    """
    # initialization for using sensor data
    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    # keeps capturing depth images until obstacle detect
    depthValue = None
    while depthValue is None or depthValue < MAX_THRESHOLD_DISTANCE_CM:
        depthValue = captureImageAndCheckForObstacle(zed, leftImage, leftDepthMatrix, runtime_params)[0]


def moveForwardUntilSignaled(motor, stop_event):
    """
    Repeatedly sends move forward commands to the robot, which is necessary to keep the robot moving overtime. Should
    be executed as a new process.

    :param motor: the Motor Controller for moving the robot
    :param stop_event: the Event object that tracks when robot receives the command to stop
    :param output_file: the file to print output to
    :param write_output_lock: lock used to protect writing output when using multiprocessing
    :param child_connection: the end of pipe to receive depth sensor data from
    """
    speed = FORWARD_SPEED
    while not stop_event.is_set():
        if motor is not None:
            motor.forward(speed)
        print("Sent move forward command with speed = {0}".format(speed))
        time.sleep(COMMAND_SEND_PERIOD_MS / MS_PER_SEC)


# ======================================================================================================================
# Depth Sensor Test Runs
# ======================================================================================================================


def initializationForTest(motor_com_port=None):
    """
    Instantiates new Motor Controller and ZED Camera object with camera opened to be used for running depth sensor
    tests. Be sure to close the Motor and Camera object when done using them.

    :param motor_com_port: com port to connect with motor, or None to not connect to motor
    :return: motor controller and ZED camera object as a 2-tuple (Motor, Camera); motor = None if enable_motor = False
    """
    # initialization
    motor = MotorController(motor_com_port) if motor_com_port else None
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.CENTIMETER

    # opens the camera
    error = zed.open(init_params)
    if error != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera. Error code:", error)
        exit(1)

    return motor, zed


def moveForwardAndStopTest(motor, zed):
    """
    Test run in which the robot moves forward and stops when it gets close enough to an obstacle. Outputs robot's
    performance to standard output and to an output file.
    """
    # one core used for capturing images, one core used for repeatedly sending move forward commands
    if multiprocessing.cpu_count() < 2:
        print("Testing error: need at least 2 CPU's")
        return

    # initialization
    stopEvent = multiprocessing.Event()

    # moves robot forward
    moveRobotForwardProcess = multiprocessing.Process(target=moveForwardUntilSignaled, args=(motor, stopEvent))
    print("Robot moving forward")
    moveRobotForwardProcess.start()

    # keeps moving forward until it sees close enough obstacle in front of it
    captureImagesUntilCloseToObstacle(zed)

    # stops robot
    stopEvent.set()
    moveRobotForwardProcess.join()
    if motor is not None:
        motor.stop()

    # continues to collect data after robot stops (to collect data on braking distance for competition design report)
    collisionFlag = False
    counter = 0  # used to periodically remind that robot is stopping in the output
    while not collisionFlag:
        depthValue = captureImageAndCheckForObstacle(zed, sl.Mat(), sl.Mat(), sl.RuntimeParameters())[0]
        if depthValue <= 0 or counter == 0:
            if depthValue > 0:
                print("Robot stopping")
            else:
                print("Robot may have collided into obstacle")
                collisionFlag = True
        counter = (counter + 1) % 10


def moveForwardAndStopTestNoMultiprocessing(motor, zed):
    # initialization for using sensor data
    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtimeParameters = sl.RuntimeParameters()

    # keeps capturing depth images until obstacle detected
    depthValue = None
    x = int(leftImage.get_width() / 2)
    y = int(leftImage.get_height() / 2)
    speed = FORWARD_SPEED
    lastTime = time.time()  # time when image was last captured
    while depthValue is None or depthValue > MIN_THRESHOLD_DISTANCE_CM:
        currTime = time.time()
        TIME_DIFF = 0.001  # time difference (in sec) for updating depthValue
        if currTime - lastTime > TIME_DIFF:
            depthValue, x, y = captureImageAndCheckForObstacle(zed, leftImage, leftDepthMatrix, runtimeParameters)
            print("Sent move forward command with speed = {0}".format(speed))
            lastTime = currTime
        if motor is not None:
            motor.forward(speed)

        # checks that it is not just noise
        if depthValue <= MIN_THRESHOLD_DISTANCE_CM:
            CONFIRM_TIMES = 10  # number of times to confirm
            countConfirm = 0
            falseAlarm = False
            while countConfirm < CONFIRM_TIMES or falseAlarm:
                depthValue, x, y = captureImageAndCheckForObstacle(zed, leftImage, leftDepthMatrix, runtimeParameters)
                if depthValue > MIN_THRESHOLD_DISTANCE_CM:
                    falseAlarm = True
                else:
                    countConfirm += 1

    # stops robot
    if motor is not None:
        motor.stop()

    # continues to collect data after robot stops (to collect data on braking distance for competition design report)
    collisionFlag = False
    counter = 0  # used to periodically remind that robot is stopping in the output
    while not collisionFlag:
        depthValue = captureImageAndCheckForObstacle(zed, sl.Mat(), sl.Mat(), sl.RuntimeParameters())[0]
        if depthValue <= 0 or counter == 0:
            if depthValue > 0:
                print("Robot stopping")
            else:
                print("Robot may have collided into obstacle")
                collisionFlag = True
        counter = (counter + 1) % 10


# ======================================================================================================================


if __name__ == "__main__":
    # initialization
    # motorForTest, zedForTest = initializationForTest()  # pass in com port as string literal to connect to motor
    motorForTest, zedForTest = initializationForTest('COM5')
    PROGRAM_START_TIME_MS = MS_PER_SEC * int(time.time())
    print("Start time:", PROGRAM_START_TIME_MS)

    # moveForwardAndStopTest(motorForTest, zedForTest)  # on successful stop, press Ctrl+C to stop program
    moveForwardAndStopTestNoMultiprocessing(motorForTest, zedForTest)
    
    # cleanup
    if motorForTest is not None:
        motorForTest.shutDown()
    zedForTest.close()

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
MIN_THRESHOLD_DISTANCE_CM = 100
MAX_THRESHOLD_DISTANCE_CM = 125

# lower and upper bounds on depth values (in cm) to consider for obstacle detection; for captureImages...() functions
MIN_OBSTACLE_DEPTH_CM = 0
MAX_OBSTACLE_DEPTH_CM = 500

# speed settings for motor controls
MAX_FORWARD_SPEED = 25
TURNING_SPEED = 10

# how often to send the same move/turn command to robot (in ms between commands) to make it do the same thing overtime
COMMAND_SEND_PERIOD_MS = 100

# TODO: delete after testing
'''
Decreases the robot's speed as it approaches obstacle
(decreases linearly from MAX_FORWARD_SPEED at MAX_OBSTACLE_DEPTH_CM to 0 at MIN_THRESHOLD_DISTANCE_CM)
'''
USE_SLOWDOWN_FEATURE = True  # Set to True to enable, or False to disable (if it doesn't work properly)

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


def captureImageAndCheckForObstacle(zed, left_image, left_depth_matrix, runtime_params, outputFile,
    writeOutputLock=None):
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
        writeToStdoutAndFile(outputFile.fileno(), "Time: {0} ms, Distance from camera at ({1}, {2}): {3} cm\n".format(
            timestampMillisecond - PROGRAM_START_TIME_MS, x, y, depthValue), writeOutputLock)
    else:
        writeToStdoutAndFile(outputFile.fileno(), "Failed to grab image. Error: {0}\n".format(error), writeOutputLock)

    return depthValue, x, y


def captureImagesUntilCloseToObstacle(zed, outputFile, writeOutputLock, parent_connection, pipe_empty_event):
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
        depthValue, x, y = captureImageAndCheckForObstacle(zed, leftImage, leftDepthMatrix, runtimeParameters,
            outputFile, writeOutputLock)
        if USE_SLOWDOWN_FEATURE:
            if pipe_empty_event.is_set():
                parent_connection.send(depthValue)
                pipe_empty_event.clear()

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


def writeToStdoutAndFile(file_descriptor, output_to_write, write_output_lock=None):
    """
    Writes output to standard output and to specified file. File associated with file_descriptor must already have
    been opened.

    :param file_descriptor: file descriptor of file to write output to
    :param output_to_write: the output to write as a String
    :param write_output_lock: lock used to protect writing output when using multiprocessing
    """
    if write_output_lock is not None:
        write_output_lock.acquire()
    sys.stdout.write(output_to_write)
    os.write(file_descriptor, output_to_write.encode())
    if write_output_lock is not None:
        write_output_lock.release()


def moveForwardUntilSignaled(motor, stop_event, output_file, write_output_lock, child_connection=None,
    pipe_empty_event=None):
    """
    Repeatedly sends move forward commands to the robot, which is necessary to keep the robot moving overtime. Should
    be executed as a new process.

    :param motor: the Motor Controller for moving the robot
    :param stop_event: the Event object that tracks when robot receives the command to stop
    :param output_file: the file to print output to
    :param write_output_lock: lock used to protect writing output when using multiprocessing
    :param child_connection: the end of pipe to receive depth sensor data from
    """
    speed = MAX_FORWARD_SPEED
    while not stop_event.is_set():
        if USE_SLOWDOWN_FEATURE and child_connection is not None and pipe_empty_event is not None:
            if not pipe_empty_event.is_set():
                depthValue = child_connection.recv()  # should not block since pipe already checked to be nonempty
                pipe_empty_event.set()

                # updates speed based on received depthValue
                speed = int((1.0 * MAX_FORWARD_SPEED) / (MAX_OBSTACLE_DEPTH_CM - MIN_THRESHOLD_DISTANCE_CM) *
                    (depthValue - MIN_THRESHOLD_DISTANCE_CM))
                if speed < 0:
                    speed = 0
                elif speed > MAX_FORWARD_SPEED:
                    speed = MAX_FORWARD_SPEED

        if motor is not None:
            motor.forward(speed)
        writeToStdoutAndFile(output_file.fileno(), "Sent move forward command {0}\n".format(speed), write_output_lock)
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
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

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
    outputFile = open("moveForwardAndStopTestResults.txt", "w")
    writeOutputLock = multiprocessing.Lock()
    stopEvent = multiprocessing.Event()
    childConnection, parentConnection = multiprocessing.Pipe(False)  # TODO: use for sharing depth value
    pipeEmptyEvent = multiprocessing.Event()
    pipeEmptyEvent.set()  # pipe is initially empty

    # moves robot forward
    moveRobotForwardProcess = multiprocessing.Process(target=moveForwardUntilSignaled, args=(motor, stopEvent,
        outputFile, writeOutputLock, childConnection, pipeEmptyEvent))
    writeToStdoutAndFile(outputFile.fileno(), "Robot moving forward\n")
    moveRobotForwardProcess.start()

    # keeps moving forward until it sees close enough obstacle in front of it
    captureImagesUntilCloseToObstacle(zed, outputFile, writeOutputLock, parentConnection, pipeEmptyEvent)

    # stops robot
    stopEvent.set()
    moveRobotForwardProcess.join()
    if motor is not None:
        motor.stop()

    # continues to collect data after robot stops (to collect data on braking distance for competition design report)
    collisionFlag = False
    counter = 0  # used to periodically remind that robot is stopping in the output
    try:
        while not collisionFlag:
            depthValue = captureImageAndCheckForObstacle(zed, sl.Mat(), sl.Mat(), sl.RuntimeParameters(), outputFile)[0]
            if depthValue <= 0 or counter == 0:
                if depthValue > 0:
                    writeToStdoutAndFile(outputFile.fileno(), "Robot stopping\n")
                else:
                    writeToStdoutAndFile(outputFile.fileno(), "Robot may have collided into obstacle\n")
                    collisionFlag = True
            counter = (counter + 1) % 10
    except KeyboardInterrupt:  # pressing Ctrl+C stops data collection if robot successfully stops at nonzero distance
        pass

    # cleanup
    outputFile.close()


def turnLeftAndStopTest(motor, zed):
    """
    Test run in which the robot turns left and stops when it detects no close obstacle in front of it.
    """
    # turns robot left
    if motor is not None:
        motor.turnLeft(TURNING_SPEED)
    print("Robot turning left")

    # keeps turning left until it sees no close obstacle in front of it
    captureImagesUntilClear(zed)

    # stops robot
    if motor is not None:
        motor.stop()
    print("Robot has stopped")


def turnRightAndStopTest(motor, zed):
    """
    Test run in which the robot turns right and stops when it detects no close obstacle in front of it.
    """
    # turns robot right
    if motor is not None:
        motor.turnRight(TURNING_SPEED)
    print("Robot turning right")

    # keeps turning right until it sees no close obstacle in front of it
    captureImagesUntilClear(zed)

    # stops robot
    if motor is not None:
        motor.stop()
    print("Robot has stopped")


def steerAwayFromObstacleTest(motor, zed):
    """
    Test run in which the robot steers away from obstacle as the robot approaches the obstacle.
    """
    # moves robot forward
    if motor is not None:
        motor.forward(MAX_FORWARD_SPEED)
    print("Robot moving forward")

    # stops robot when close to obstacle
    obstacleX, obstacleY, imageWithCloseObstacle = captureImagesUntilCloseToObstacle(zed)
    if motor is not None:
        motor.stop()
    print("Robot has stopped")

    # chooses direction in which to turn robot to dodge obstacle
    centerX = imageWithCloseObstacle.get_width() / 2
    if obstacleX < centerX:  # obstacle on left side
        if motor is not None:
            motor.turnRight(TURNING_SPEED)
        print("Robot turning right")
    else:  # obstacle on right side
        if motor is not None:
            motor.turnLeft(TURNING_SPEED)
        print("Robot turning left")

    # stops robot when it is clear
    captureImagesUntilClear(zed)
    if motor is not None:
        motor.stop()
    print("Robot has stopped")

    # moves robot forward
    if motor is not None:
        motor.forward(MAX_FORWARD_SPEED)
    print("Robot moving forward")

    # stops robot when it is clear
    captureImagesUntilCloseToObstacle(zed)
    if motor is not None:
        motor.stop()
    print("Robot has stopped")


def zigzagDownCorridorTest(motor, zed):
    """
    Test run in which robot navigates down corridor with small obstacles in the way
    """
    # initialization
    tracking_parameters = sl.PositionalTrackingParameters()
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    zed_pose = sl.Pose()
    runtime_parameters = sl.RuntimeParameters()

    # gets initial pose data
    initialTranslationData = None
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # gets the pose of the left eye of the camera with reference to the world frame
        zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
        initialTranslation = zed_pose.get_translation()
        initialTranslationData = initialTranslation.get()

    # lets robot travel until it is at least 1000 cm away from start position
    distanceFromStart = 0
    while distanceFromStart < 1000:
        # moves robot forward
        if motor is not None:
            motor.forward(MAX_FORWARD_SPEED)
        print("Robot moving forward")

        # stops robot when close to obstacle
        obstacleX, obstacleY, imageWithCloseObstacle = captureImagesUntilCloseToObstacle(zed)
        if motor is not None:
            motor.stop()
        print("Robot has stopped")

        # chooses direction in which to turn robot to dodge obstacle
        centerX = imageWithCloseObstacle.get_width() / 2
        if obstacleX < centerX:  # obstacle on left side
            if motor is not None:
                motor.turnRight(TURNING_SPEED)
            print("Robot turning right")
        else:  # obstacle on right side
            if motor is not None:
                motor.turnLeft(TURNING_SPEED)
            print("Robot turning left")

        # stops robot when it is clear
        captureImagesUntilClear(zed)
        if motor is not None:
            motor.stop()
        print("Robot has stopped")

        # updates distance from start
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            currentTranslation = zed_pose.get_translation()
            currentTranslationData = currentTranslation.get()
            distanceFromStart = abs(initialTranslationData[0] - currentTranslationData[0])
            print("Distance from start: {0}".format(distanceFromStart))


# TODO: delete later
def miscellaneousFileWritingAndMultiprocessingTest():
    """
    Test writing to stdout and file with multiprocessing. Output printed in "sample.txt".
    """
    file = open("sample.txt", "w")
    lock = multiprocessing.Lock()
    event = multiprocessing.Event()
    process = multiprocessing.Process(target=moveForwardUntilSignaled, args=(None, event, file, lock))
    process.start()
    for i in range(3, 0, -1):
        writeToStdoutAndFile(file.fileno(), "{0} sec left...\n".format(i), lock)
        time.sleep(1)
    event.set()
    process.join()
    writeToStdoutAndFile(file.fileno(), "Time's up!\n")
    file.close()


# ======================================================================================================================


if __name__ == "__main__":
    # TODO: delete later
    # miscellaneousFileWritingAndMultiprocessingTest()

    # initialization
    motorForTest, zedForTest = initializationForTest()  # pass in com port as string literal to connect to motor
    PROGRAM_START_TIME_MS = MS_PER_SEC * int(time.time())
    print("Start time:", PROGRAM_START_TIME_MS)

    moveForwardAndStopTest(motorForTest, zedForTest)  # on successful stop, press Ctrl+C to stop program
    # turnLeftAndStopTest(motorForTest, zedForTest)
    # turnRightAndStopTest(motorForTest, zedForTest)

    # steerAwayFromObstacleTest(motorForTest, zedForTest)
    # zigzagDownCorridorTest(motorForTest, zedForTest)

    # cleanup
    if motorForTest is not None:
        motorForTest.shutDown()
    zedForTest.close()

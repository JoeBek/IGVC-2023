"""
Obstacle avoidance tests using depth sensor.

Download Python ZED SDK by following this tutorial: https://www.stereolabs.com/docs/app-development/python/install

Author: Harrison Bui
Date: February 14, 2024
"""

from MotorControlAPI import MotorController
import pyzed.sl as sl
import math

# how close robot should be allowed to approach obstacle (in cm); for captureImages...() functions
MIN_THRESHOLD_DISTANCE = 100
MAX_THRESHOLD_DISTANCE = 125

# lower and upper bounds on depth values (in cm) to consider for obstacle detection; for captureImages...() functions
MIN_OBSTACLE_DEPTH = 0
MAX_OBSTACLE_DEPTH = 300

# speed settings for motor controls
FORWARD_SPEED = 25
TURNING_SPEED = 10


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
    Detects approximate point on center line of image for any obstacle that may be close.

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
        currentDepth = clipData(currentDepth, MIN_OBSTACLE_DEPTH, MAX_OBSTACLE_DEPTH)
        depthList.append(currentDepth)

    # finds leftmost pixel of obstacle
    leftBoundX = 0
    while leftBoundX < len(depthList) and depthList[leftBoundX] >= MAX_OBSTACLE_DEPTH:
        leftBoundX += 1

    # checks if no obstacle detected
    if leftBoundX >= len(depthList):
        return None

    # finds rightmost pixel of obstacle
    rightBoundX = len(depthList) - 1
    while rightBoundX >= 0 and depthList[rightBoundX] >= MAX_OBSTACLE_DEPTH:
        rightBoundX -= 1

    # gets center pixel between the two boundary pixels
    centerOfObstacleX = int((leftBoundX + rightBoundX) / 2)
    return centerOfObstacleX, centerY


def captureImagesUntilCloseToObstacle(zed):
    """
    Uses depth sensor to wait until a close obstacle is detected. Used to avoid colliding into obstacles.

    :param zed: the ZED camera whose depth sensor to use

    :return: data of frame in which close obstacle detected as a 3-tuple (x, y, image)
    """
    # initialization for using sensor data
    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    # keeps capturing depth images until obstacle detect
    depthValue = MIN_THRESHOLD_DISTANCE + 10
    x = int(leftImage.get_width() / 2)
    y = int(leftImage.get_height() / 2)
    while depthValue > MIN_THRESHOLD_DISTANCE:
        # grabs an image
        error = zed.grab(runtime_params)
        if error == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(leftImage, sl.VIEW.LEFT)  # gets left image
            zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image

            # gets depth value of obstacle, if any
            obstacleCoordinates = getCoordinatesOfCloseObstacle(leftImage, leftDepthMatrix)
            if obstacleCoordinates is None:
                x = int(leftImage.get_width() / 2)
                y = int(leftImage.get_height() / 2)
            else:
                x, y = obstacleCoordinates
            err, depthValue = leftDepthMatrix.get_value(x, y)
            depthValue = clipData(depthValue, MIN_OBSTACLE_DEPTH, MAX_OBSTACLE_DEPTH)
            print("Distance from camera at ({0}, {1}): {2} cm".format(x, y, depthValue))
        else:
            print("Failed to grab image. Error:", error)

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
    depthValue = MAX_THRESHOLD_DISTANCE - 10
    while depthValue < MAX_THRESHOLD_DISTANCE:
        # grabs an image
        error = zed.grab(runtime_params)
        if error == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(leftImage, sl.VIEW.LEFT)  # gets left image
            zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image

            # gets depth value of obstacle, if there is an obstacle
            obstacleCoordinates = getCoordinatesOfCloseObstacle(leftImage, leftDepthMatrix)
            if obstacleCoordinates is None:
                x = int(leftImage.get_width() / 2)
                y = int(leftImage.get_height() / 2)
            else:
                x, y = obstacleCoordinates
            err, depthValue = leftDepthMatrix.get_value(x, y)
            depthValue = clipData(depthValue, MIN_OBSTACLE_DEPTH, MAX_OBSTACLE_DEPTH)
            print("Distance from camera at ({0}, {1}): {2} cm".format(x, y, depthValue))
        else:
            print("Failed to grab image. Error:", error)


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
    Test run in which the robot moves forward and stops when it gets close enough to an obstacle.
    """
    # moves robot forward
    if motor is not None:
        motor.forward(FORWARD_SPEED)
    print("Robot moving forward")

    # keeps moving forward until it sees close enough obstacle in front of it
    captureImagesUntilCloseToObstacle(zed)

    # stops robot
    if motor is not None:
        motor.stop()
    print("Robot has stopped")


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
        motor.forward(FORWARD_SPEED)
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
        motor.forward(FORWARD_SPEED)
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
            motor.forward(FORWARD_SPEED)
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


# ======================================================================================================================


if __name__ == "__main__":
    # initialization
    motorForTest, zedForTest = initializationForTest()  # pass in com port as string literal to connect to motor

    moveForwardAndStopTest(motorForTest, zedForTest)
    # turnLeftAndStopTest(motorForTest, zedForTest)
    # turnRightAndStopTest(motorForTest, zedForTest)

    # steerAwayFromObstacleTest(motorForTest, zedForTest)
    # zigzagDownCorridorTest(motorForTest, zedForTest)

    # cleanup
    if motorForTest is not None:
        motorForTest.shutDown()
    zedForTest.close()

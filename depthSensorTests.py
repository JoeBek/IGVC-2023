"""
Obstacle avoidance tests using depth sensor.

Authors: Harrison Bui
Date: February 13, 2024
"""

import pyzed.sl as sl
import math

# how close robot should be allowed to approach obstacle (in cm)
THRESHOLD_DISTANCE = 100

# lower and upper bounds on depth values (in cm) to consider for obstacle detection
MIN_OBSTACLE_DEPTH = 0
MAX_OBSTACLE_DEPTH = 300


def clipData(data_to_fix, lower_bound, upper_bound):
    """
    Determines new value for given value so that it stays within given bounds. Values outside of bounds get reassigned
    to the closest bound value. Function is used to clean up noise in depth sensor data.

    :param data_to_fix: float value to recompute from
    :param lower_bound: the lowest acceptable value for value
    :param upper_bound: the highest acceptable value for value

    :return: the updated data value as a float
    """
    if data_to_fix > upper_bound or math.isnan(data_to_fix):
        return upper_bound
    elif data_to_fix < lower_bound:
        return lower_bound
    return data_to_fix


def detectCloseObstacle(image, depth_matrix):
    """
    Detects approximate point on center line of image of any obstacle that may be close.

    :param image: image retrieved using ZED SDK
    :param depth_matrix: depth measurement retrieved using ZED SDK (in cm)

    :return: center of obstacle (x, y) as 2-tuple, or None if no obstacle detected
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


def moveForwardAndStopTest():
    """
    Does a test run where the robot moves forward and stops when it gets close enough to an obstacle.
    """
    # creates a ZED camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.CENTIMETER

    # opens the camera
    error = zed.open(init_params)
    if error != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera. Error code:", error)
        exit(1)

    # moves robot forward
    # TODO: replace with James' API
    print("Moving forward")

    # initialization for using sensor data
    leftImage = sl.Mat()
    leftDepth = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    # keeps moving robot forward until close enough to obstacle
    depthValue = THRESHOLD_DISTANCE + 10
    while depthValue > THRESHOLD_DISTANCE:
        # grabs an image
        error = zed.grab(runtime_params)
        if error == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(leftImage, sl.VIEW.LEFT)  # gets left image
            zed.retrieve_measure(leftDepth, sl.MEASURE.DEPTH)  # gets left depth image

            # gets depth value of obstacle, if there is an obstacle
            obstacleCoordinates = detectCloseObstacle(leftImage, leftDepth)
            if obstacleCoordinates is None:
                x = int(leftImage.get_width() / 2)
                y = int(leftImage.get_height() / 2)
            else:
                x, y = obstacleCoordinates
            err, depthValue = leftDepth.get_value(x, y)
            depthValue = clipData(depthValue, MIN_OBSTACLE_DEPTH, MAX_OBSTACLE_DEPTH)
            print("Distance to Camera at ({0}, {1}): {2} cm".format(x, y, depthValue))
        else:
            print("Failed to grab image. Error:", error)

    # stops robot
    # TODO: replace with James' API
    print("Stopped")

    # closes the camera
    zed.close()


if __name__ == "__main__":
    moveForwardAndStopTest()

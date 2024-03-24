"""
Module for obstacle detection using ZED depth sensor.

Author: Harrison Bui
Date: March 23, 2024
"""

import pyzed.sl as sl
import math

OBSTACLE_AVOIDANCE_DEPTH_CM = 300  # distance (in cm) at which robot attempts to avoid obstacle
MAX_DETECTION_DEPTH_CM = 400  # maximum distance (in cm) used for object detection


# TODO: attempt for detecting multiple objects
def findCoordinatesOfCloseObstacles(depth_list):
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


def testFindCoordinatesOfCloseObstacles():
    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[0, 1, 2, 3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[1, 2, 3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[0], [2, 3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[0, 1], [3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = [[0, 1, 2]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[2, 3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[1], [3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = [[1, 2]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[0], [3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = [[0], [2]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = [[0, 1]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM - 1]
    expected = [[3]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = [[2]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = [[1]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM - 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = [[0]]
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)

    testDepthList = [OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1, OBSTACLE_AVOIDANCE_DEPTH_CM + 1,
                     OBSTACLE_AVOIDANCE_DEPTH_CM + 1]
    expected = []
    actual = findCoordinatesOfCloseObstacles(testDepthList)
    assert expected == actual, "Input: {0}\nExpected: {1}\nActual: {2}\n".format(testDepthList, expected, actual)


# ======================================================================================================================
# Obstacle Detection
# ======================================================================================================================
if __name__ == "__main__":
    # testFindCoordinatesOfCloseObstacles()

    # setup for zed camera (for obstacle detection)
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.CENTIMETER
    error = zed.open(init_params)
    if error != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera. Error code:", error)
        exit(1)

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
            if currentDepth > MAX_DETECTION_DEPTH_CM:
                currentDepth = MAX_DETECTION_DEPTH_CM
            elif math.isnan(currentDepth):
                currentDepth = MAX_DETECTION_DEPTH_CM
            elif currentDepth < 0:
                currentDepth = 0
            depthList.append(currentDepth)

        obstacleLocationList = findCoordinatesOfCloseObstacles(depthList)
        print("Depth values:", depthList)
        print("Result:", obstacleLocationList)
        print("Number of objects:", len(obstacleLocationList))

    zed.close()

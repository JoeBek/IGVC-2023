"""
Module for controlling robot autonomously based on object and line detection.

Authors: Harrison, Shreya
Date: March 23, 2024
"""

from enum import Enum


class Direction(Enum):
    """
    Directions robot can head toward using motor controls
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
    sampleDirectionToMove = Direction.LEFT
    sampleDistanceFromLine_cm = 100  # in centimeters
    return sampleDirectionToMove, sampleDistanceFromLine_cm


# ======================================================================================================================
# Obstacle Detection
# ======================================================================================================================

# TODO: implement avoidObstacle()
def avoidObstacle():
    """
    Decides what direction the robot should go based on any obstacle detected.

    :return: direction to move toward (as a Direction enum), distance from obstacle in centimeters (as a float).
    """
    sampleDirectionToMove = Direction.RIGHT
    sampleDistanceFromObject_cm = 200  # in centimeters
    return sampleDirectionToMove, sampleDistanceFromObject_cm


# ======================================================================================================================
# Combined
# ======================================================================================================================

def avoidLineAndObstacle():
    directionAwayFromLine, distanceFromLine = avoidLine()  # direction to move away from line
    directionAwayFromObstacle, distanceFromObstacle = avoidObstacle()  # direction to move away from obstacle

    directionToMove = Direction.FORWARD
    if directionAwayFromLine == directionAwayFromObstacle:
        directionToMove = directionAwayFromLine
    elif directionAwayFromLine == Direction.FORWARD:
        directionToMove = directionAwayFromObstacle
    elif directionAwayFromObstacle == Direction.FORWARD:
        directionToMove = directionAwayFromLine
    elif ((directionAwayFromLine == Direction.LEFT and directionAwayFromObstacle == Direction.RIGHT) or
          (directionAwayFromLine == Direction.RIGHT and directionAwayFromObstacle == Direction.LEFT)):
        if distanceFromObstacle < distanceFromLine:
            directionToMove = directionAwayFromObstacle
        else:  # distanceFromLine <= distanceFromObstacle
            directionToMove = directionAwayFromLine

    print("Direction to move: {0},\n\tLine: ({1}, {2}),\n\tObstacle: ({3}, {4})".format(directionToMove.value,
        directionAwayFromLine.value, distanceFromLine, directionAwayFromObstacle.value, distanceFromObstacle))
    return directionToMove


if __name__ == "__main__":
    while True:
        decidedDirectionToMove = avoidLineAndObstacle()
        # TODO: integrate with motor controls

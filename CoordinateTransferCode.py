# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
import pyzed.sl as sl
import math
import numpy as np
import cv2
import os
from MotorControlAPI import MotorController
from GPS_API import GPS

#Defining Important Constants
DEPTH_VALUE = 300 # when objects are closer than DEPTH_VALUE, the robot will turn to avoid them
MAX_DEPTH = 400
ROBOT_SPEED = 20
ONE_SECOND_DELAY = 1000000000
ROBOT_WIDTH = 100 #Horizontal width of robot in cm

WAYPOINT_LATITUDE = 37.228889
WAYPOINT_LONGITUDE = -80.4155556

"""
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
        for i in range(len(obstacleLocationList)-1, 0, -1):
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
"""

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
    else:
        return currentDirection
    
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

#--------------------------------------------------------------
# LINE DETECTION FUNCTIONS
#--------------------------------------------------------------

def display_images(images, cmap=None):
    plt.figure(figsize=(40,40))    
    for i, image in enumerate(images):
        plt.subplot(3,2,i+1)
        plt.imshow(image, cmap)
        plt.autoscale(tight=True)
    plt.show()
    
#display_images(imageList)


def color_filter(image):
    #convert from RGB to HLS (Hue, Lightness, Saturation)
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    lower = np.array([0,190,0])
    upper = np.array([255,255,255])

 
    whitemask = cv2.inRange(hls, lower, upper)
    
    masked = cv2.bitwise_and(image, image, mask = whitemask)    
    


    return masked

#filtered_img = list(map(color_filter, imageList))

#display_images(filtered_img)


def roi(img):
    #function to idenify region of interest, using a triangle to focus on where the lines are
    if img is not None:
        x = int(img.shape[1])
        y = int(img.shape[0])
        shape = np.array([[int(0), int(y)], [int(x), int(y)], [int(0.55*x), int(0.6*y)], [int(0.45*x), int(0.6*y)]])

        mask = np.zeros_like(img)

        #Uses 3 channels or 1 channel for color depending on input image
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        #creates a polygon with the mask color (for area between lines)
        cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)

        #returns the image only where the mask pixels are not zero
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    else: 
        print("None")

#roi_img = list(map(roi, filtered_img))

#display_images(roi_img)


def grayscale(img):
    #canny needs a gray image, so we convert
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img):
    #using canny to get edges
    return cv2.Canny(grayscale(img), 50, 120)

#canny_img = list(map(canny, roi_img))
#display_images(canny_img,cmap='gray')

rightSlope, leftSlope, rightIntercept, leftIntercept = [],[],[],[]
def draw_lines(img, lines, thickness=5):
    global rightSlope, leftSlope, rightIntercept, leftIntercept

    rightColor=[0,255,0]
    leftColor=[255,0,0]
    
    #this is used to filter out the outlying lines that can affect the average
    #We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    if lines is not None: 
        for line in lines:
            for x1,y1,x2,y2 in line:
                slope = (y1-y2)/(x1-x2)
                if slope > 0.3:
                    if x1 > 500 :
                        yintercept = y2 - (slope*x2)                    
                        rightSlope.append(slope)
                        rightIntercept.append(yintercept)
                    else: None                
                elif slope < -0.3:
                    if x1 < 600:
                        yintercept = y2 - (slope*x2)                    
                        leftSlope.append(slope)
                        leftIntercept.append(yintercept)    
        #print("works")
    else:
        #print("was None")
        hello = 0
                    
                    
    #We use slicing operators and np.mean() to find the averages of the 30 previous frames
    #This makes the lines more stable, and less likely to glitch
    leftavgSlope = np.mean(leftSlope[-30:])
    leftavgIntercept = np.mean(leftIntercept[-30:])
    
    rightavgSlope = np.mean(rightSlope[-30:])
    rightavgIntercept = np.mean(rightIntercept[-30:])
    
    left_line_x1 = None 
    right_line_x1 = None
    #plotting the lines
    try:
        left_line_x1 = int((0.65*img.shape[0] - leftavgIntercept)/leftavgSlope)
        left_line_x2 = int((img.shape[0] - leftavgIntercept)/leftavgSlope)
    
        right_line_x1 = int((0.65*img.shape[0] - rightavgIntercept)/rightavgSlope)
        right_line_x2 = int((img.shape[0] - rightavgIntercept)/rightavgSlope)

        pts = np.array([[left_line_x1, int(0.65*img.shape[0])],[left_line_x2, int(img.shape[0])],[right_line_x2, int(img.shape[0])],[right_line_x1, int(0.65*img.shape[0])]], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.fillPoly(img,[pts],(0,0,255))      
        
        
        cv2.line(img, (left_line_x1, int(0.65*img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
        cv2.line(img, (right_line_x1, int(0.65*img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)
    except ValueError:
            #I keep getting errors for some reason, so I put this here. Idk if the error still persists.
        pass

    return [left_line_x1, right_line_x1]
    
    
                
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    """
    #using hough to get the lines from the canny image
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    coors = draw_lines(line_img, lines)
    #line_img2 = cv2.circle(line_img, (coors[0], 400), radius=10, color=(255, 255, 255), thickness=-6)
    #line_img2 = cv2.circle(line_img2, (coors[1], 400), radius=10, color=(255, 255, 255), thickness=-6)
    direction = "up"
    """
    if coors[0] is not None and coors[1] is not None: 
        left_diff = int(coors[0])
        right_diff = 635-int(coors[1])
        padding = 20
        #print("left: ", left_diff)
        #print("right: ", right_diff)
        # r = 400 
        if (right_diff -padding < left_diff) and (left_diff < right_diff + padding):
            print("stay straight")
            direction = "up"
        elif (left_diff < right_diff - padding) :
            print("move left")
            direction = "left"
        elif (left_diff > right_diff + padding): 
            print("move right")
            direction = "right"

        '''if (left_diff < right_diff - padding) or (left_diff > right_diff + padding): 
            print("move right")
            #myVar.turnRight(20)
        elif (right_diff < left_diff - padding) or (right_diff > left_diff + padding): 
            print("move left")
            #myVar.turnLeft(20)
        elif left_diff == right_diff: 
            print("stay straight")
            #myVar.forward(20)'''

    #initialTime = sendMotorCommand(motors, direction, initialTime)
    """
    
    return line_img


def linedetect(img):
    return hough_lines(img, 1, np.pi/180, 10, 20, 100).first

#hough_img = list(map(linedetect, canny_img))
#display_images(hough_img)

def processImage(image):
    #rgbImg = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)
    rgbImg = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)      #COLOR_BGRA2BGR

    #function to combine all previous functions
    interest = roi(image)
    
    filterimg = color_filter(interest)

    canny = cv2.Canny(grayscale(filterimg), 50, 120)

    myline = hough_lines(canny, 1, np.pi/180, 10, 20, 5)

    weighted_img = cv2.addWeighted(myline, 1, rgbImg, 0.8, 0)
    
    #cv2.imshow("img", weighted_img)


    ''' motorObj = MotorController('COM4')
    initialTime = time.time_ns()
    initialTime = sendMotorCommand(motorObj, command, initialTime)'''
    
    return weighted_img, myline

def findCoordinatesOfObstacles(zed, lineImg):
    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    error = zed.grab(runtime_params)
    zed.retrieve_image(leftImage, sl.VIEW.LEFT)
    zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image

    obstacleList = []

    LINE_COLOR_RIGHT = [0,255,0]
    LINE_COLOR_LEFT = [255,0,0]
    HALF_FOV_DEGREES_VERT = 45
    CENTER_PIXEL_VERT = (leftImage.get_height() / 2)
    HALF_FOV_DEGREES_HORIZ = 55
    CENTER_PIXEL_HORIZ = (leftImage.get_width() / 2)

    OBSTACLE_THRESHOLD = 300

    for j in range(leftImage.get_height()):
        for i in range(leftImage.get_width()):
            error, currentDepth = leftDepthMatrix.get_value(i, j)
            if lineImg.get_value(i,j) == LINE_COLOR_RIGHT or lineImg.get_value(i,j) == LINE_COLOR_LEFT:    #If the cooresponding image pixel is a line, then that depth value is an obstacle
                #Convert depth + pixel value into overhead x,y coordinates

                phi_degrees = 1.000 * HALF_FOV_DEGREES_VERT * (j - CENTER_PIXEL_VERT)/CENTER_PIXEL_VERT
                phi_radians =  phi_degrees * math.pi/180 # Angle to object in radians
                correctedDepth = currentDepth * math.cos(phi_radians)

                theta_degrees = 1.000 * HALF_FOV_DEGREES_HORIZ * (currentX - CENTER_PIXEL_HORIZ)/CENTER_PIXEL_HORIZ #Angle to object in degrees
                theta_radians =  theta_degrees * math.pi/180 # Angle to object in radians

                #Find overhead X coordinate
                overheadXPos = correctedDepth * math.sin(theta_radians)

                #Find overhead Y coordinate
                overheadYPos = correctedDepth * math.cos(theta_radians)
                #Add coordinate to obstacle list

                if overheadYPos < OBSTACLE_THRESHOLD:
                    obstacleList.append({overheadXPos, overheadYPos})
                


    centerY = int(leftImage.get_height() / 2)
    for currentX in range(leftImage.get_width()):
        error, currentDepth = leftDepthMatrix.get_value(currentX, centerY)
        # cleans up data
        """
        if (currentDepth > MAX_DEPTH):
            currentDepth = MAX_DEPTH
        elif math.isnan(currentDepth):
            currentDepth = MAX_DEPTH
        elif (currentDepth < 0):
            currentDepth = 0
        """

        if not math.isnan(currentDepth): # depth value is nan when the distance is too far away

            #Find overhead x,y coordinates of obstacle points
            #Add coordinates to obstacle list
            #Find theta
            theta_degrees = 1.000 * HALF_FOV_DEGREES_HORIZ * (currentX - CENTER_PIXEL_HORIZ)/CENTER_PIXEL_HORIZ #Angle to object in degrees
            theta_radians =  theta_degrees * math.pi/180 # Angle to object in radians

            #Find overhead X coordinate
            overheadXPos = currentDepth * math.sin(theta_radians)

            #Find overhead Y coordinate
            overheadYPos = currentDepth * math.cos(theta_radians)

            if overheadYPos < OBSTACLE_THRESHOLD:
                obstacleList.append({overheadXPos, overheadYPos})


        
    #Return obstacle list
    return obstacleList

def comparePathToObstacles(coordinateList):
    ANGLE_INCREMENT = 1

    angle = 0.0001

    minDistanceRight = 100000
    minDistanceLeft = 100000
    while minDistanceRight > ROBOT_WIDTH/2 and minDistanceLeft > ROBOT_WIDTH/2:
        angle_radians =  angle * math.pi/180 # Angle to object in radians
        slope = math.atan(angle)
        a = -1 * slope

        minDistanceRight = 10000
        for coord in coordinateList:
            distance = math.abs(a * coord[0] + coord[1])/math.sqrt(a^2 + 1)
            if (distance < minDistanceRight):
                minDistanceRight = distance


        a = slope

        minDistanceLeft = 10000
        for coord in coordinateList:
            distance = math.abs(a * coord[0] + coord[1])/math.sqrt(a^2 + 1)
            if (distance < minDistanceLeft):
                minDistanceLeft = distance
        
        angle += ANGLE_INCREMENT

    if (minDistanceRight < minDistanceLeft):
        return angle
    
    return -1 * angle

# Perform all autonomous navigation checks
# Returns the direction the robot should move according to the object detection and line detection
# Should only return "up", "left", or "right"
def runAutonomousControls(zed):
    #depthSensorDirection = runDepthSensor(zed) # Finds the best course of action according to the depth sensor

    # Get the image from the zed camera
    leftImage1 = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    error = zed.grab(runtime_params)
    zed.retrieve_image(leftImage1, sl.VIEW.LEFT)

    frame = leftImage1.get_data()
    final, lineImg = processImage(frame)  # process the current image and color the lines in a solid color

    cv2.imshow("img", final)

    # Finds the overhead x,y coordinates of all obstacles and lines
    coordinateList = findCoordinatesOfObstacles(zed, lineImg)

    # Compare all obstacles points to current path
    desiredAngle = comparePathToObstacles(coordinateList)


    return desiredAngle


#----------------------------------------------------------------------
# Setting up usb connections
#----------------------------------------------------------------------
#motors = MotorController('COM4')

BluetoothSerialObj = serial.Serial('COM3') # COMxx  format on Windows
                  # ttyUSBx format on Linux
BluetoothSerialObj.baudrate = 9600  # set Baud rate to 9600
BluetoothSerialObj.bytesize = 8   # Number of data bits = 8
BluetoothSerialObj.parity  ='N'   # No parity
BluetoothSerialObj.stopbits = 1   # Number of Stop bits = 1

# Setting up Zed camera for both line detection and object detection
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.CENTIMETER
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
error = zed.open(init_params)


#----------------------------------------------------------------------
# Setup of important variables
#----------------------------------------------------------------------

# Holds the time the last motor command was sent
# Used to ensure that motor commands are not sent more than 10 times per second
timeOfLastCommand = time.time_ns()

# Holds the command about to be sent
currentCommand = "stop"

# Boolean to keep track of whether the robot is in autonomous mode, starts in manual control mode
doAutonomous = False

#----------------------------------------------------------------------
# Main loop
#----------------------------------------------------------------------
while True:

    # Check for a bluetooth command
    if (BluetoothSerialObj.inWaiting() > 0):
        BluetoothString = BluetoothSerialObj.readline()
    else:
        BluetoothString = ""

    # Always parse the bluetooth command even if the robot is in autonomous mode to look out for an stop command
    manualControlCommand = parseManualControls(BluetoothString, currentCommand)

    if (manualControlCommand == "shutdown"):
        print("Program Ended")
        #motors.shutDown()
        break
    elif (manualControlCommand == "autonomous"):
        BluetoothSerialObj.write(b"Z\n")
        doAutonomous = not doAutonomous
    elif (~doAutonomous): #Only set the currentKey to the manual command if the robot is not in autonomous mode
        currentCommand = manualControlCommand
    else:
        currentCommand = "stop"

    # Always be collecting data for autonomous controls
    autonomousCommand = runAutonomousControls(zed)


    print("Desired Angle:")
    print(autonomousCommand)

    # if the robot is in autonomous mode, set the current command to the best course of action according to the autonomous control logic
    #if (doAutonomous):
    #    currentCommand = autonomousCommand
        

    # send the next course of action to the motors
    #timeOfLastCommand = sendMotorCommand(motors, currentCommand, timeOfLastCommand)

    # This is required, I don't know why but opencv doesn't work without this if statement contained within the main loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Shut down everything and close open ports
#motors.shutDown()
BluetoothSerialObj.close()      # Close the port
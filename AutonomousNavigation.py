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

# Importing pygame module
import pygame
from pygame.locals import *


#Defining Important Constants
DEPTH_VALUE = 300 # when objects are closer than DEPTH_VALUE, the robot will turn to avoid them
MAX_DEPTH = 400
ROBOT_SPEED = 20
ONE_SECOND_DELAY = 1000000000
ROBOT_WIDTH = 50 #Horizontal width of robot in cm

WAYPOINT_LATITUDE = 37.228889
WAYPOINT_LONGITUDE = -80.4155556

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
    
def sendMotorCommand(motorObj, command, lastCommandTime, leftSpeed=0, rightSpeed=0):
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
            elif (command == "custom"): #Defaults to stopping if not left and right speed are given
                print("Custom speed")
                motorObj.customMovement(leftSpeed, rightSpeed)
            elif (command == "stop"):
                print("Stopping")
                motorObj.stop()
    return lastCommandTime

#--------------------------------------------------------------
# LINE DETECTION FUNCTIONS
#--------------------------------------------------------------

    
#display_images(imageList)


def color_filter(image):
    #convert from RGB to HLS (Hue, Lightness, Saturation)
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    lower = np.array([0,190,0])
    upper = np.array([255,255,255])

    whitemask = cv2.inRange(hls, lower, upper)
    
    masked = cv2.bitwise_and(image, image, mask = whitemask)    
    
    return masked


def roi(img):
    #function to idenify region of interest, using a triangle to focus on where the lines are
    if img is not None:
        x = int(img.shape[1])
        y = int(img.shape[0])
        shape = np.array([[int(0), int(y-100)], [int(x), int(y-100)], [int(0.55*x), int(0.6*y)], [int(0.45*x), int(0.6*y)]])

        mask = np.zeros_like(img)

        #Uses 3 channels or 1 channel for color depending on input imageq
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


def grayscale(img):
    #canny needs a gray image, so we convert
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img):
    #using canny to get edges
    return cv2.Canny(grayscale(img), 100, 150)


rightSlope, leftSlope, rightIntercept, leftIntercept = [],[],[],[]
prevRightSlope = 0
prevLeftSlope = 0
def draw_lines(img, lines):
    global rightSlope, leftSlope, rightIntercept, leftIntercept, prevRightSlope, prevLeftSlope

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

    """
    if (time.time_ns() > lastCommandTime + (ONE_SECOND_DELAY*0.1)):
        if prevLeftSlope == leftavgSlope:
            print("stop")
            return [left_line_x1, right_line_x1, leftavgSlope, rightavgSlope]
        elif prevRightSlope == rightavgSlope:
            print("stop")
            return [left_line_x1, right_line_x1, leftavgSlope, rightavgSlope]
        """
    
    #plotting the lines
    
    left_line_x1 = None 
    right_line_x1 = None
    #plotting the lines
    try:
        if math.isnan(rightavgSlope):
       #draw left line and everything to the right
            left_line_x1 = int((0.65*img.shape[0] - leftavgIntercept)/leftavgSlope)
            left_line_x2 = int((img.shape[0] - leftavgIntercept)/leftavgSlope)

            pts = np.array([[left_line_x1, int(0.65*img.shape[0])],[left_line_x2, int(img.shape[0])],[int(img.shape[1]), int(img.shape[0])],[int(img.shape[1]), int(0.5*img.shape[1])]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.fillPoly(img,[pts],(0,0,255))  

            cv2.line(img, (left_line_x1, int(0.65*img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
            return [left_line_x1, float("nan"), leftavgSlope, float("nan")]

        '''if math.isnan(leftavgSlope):
        #draw right line and everything to the left
            right_line_x1 = int((0.65*img.shape[0] - rightavgIntercept)/rightavgSlope)
            right_line_x2 = int((img.shape[0] - rightavgIntercept)/rightavgSlope)

            pts = np.array([[15, int(0.65*img.shape[0])],[0, int(img.shape[0])],[right_line_x2, int(img.shape[0])],[right_line_x1, int(0.65*img.shape[0])]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.fillPoly(img,[pts],(0,0,255)) 

            cv2.line(img, (right_line_x1, int(0.65*img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)
            return [float("nan"), right_line_x1, float("nan"), rightavgSlope]'''
        left_line_x1 = int((0.65*img.shape[0] - leftavgIntercept)/leftavgSlope)
        left_line_x2 = int((img.shape[0] - leftavgIntercept)/leftavgSlope)
    
        right_line_x1 = int((0.65*img.shape[0] - rightavgIntercept)/rightavgSlope)
        right_line_x2 = int((img.shape[0] - rightavgIntercept)/rightavgSlope)

        pts = np.array([[left_line_x1, int(0.65*img.shape[0])],[left_line_x2, int(img.shape[0])],[right_line_x2, int(img.shape[0])],[right_line_x1, int(0.65*img.shape[0])]], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.fillPoly(img,[pts],(0,0,255))      
        
        
        cv2.line(img, (left_line_x1, int(0.65*img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 20)
        #cv2.line(img, (right_line_x1, int(0.65*img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 20)
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

    blur = cv2.GaussianBlur(grayscale(filterimg), (5,5),0)

    canny = cv2.Canny(blur, 100, 150)

    myline = hough_lines(canny, 1, np.pi/180, 10, 20, 5)

    weighted_img = cv2.addWeighted(myline, 1, rgbImg, 0.8, 0)
    
    cv2.imshow("img",canny)
    cv2.imshow("final",weighted_img)



    ''' motorObj = MotorController('COM4')
    initialTime = time.time_ns()
    initialTime = sendMotorCommand(motorObj, command, initialTime)'''
    
    return weighted_img, myline

def coordinateTransform(xpos, ypos):
    transformedXpos = int(300 + xpos)
    transformedYpos = int(600 - ypos)

    return [transformedXpos, transformedYpos]

def getDepthMatrixValue(depthList, xVal, yVal):
    
    average = 0
    for matrix in depthList:
        average += matrix.get_value(xVal, yVal)

    average = average/len(depthList)

    return average

prevDepthMatrixList = []

def findCoordinatesOfObstacles(zed, lineImg):
    global prevDepthMatrixList

    leftImage = sl.Mat()
    leftDepthMatrix = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    error = zed.grab(runtime_params)
    zed.retrieve_image(leftImage, sl.VIEW.LEFT)
    zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image

    # Add the current depth matrix to the list of previous depth matrix measurements
    # Enables depth value measurements to be averaged over the last couple of frames
    prevDepthMatrixList.append(leftDepthMatrix)

    if (len(prevDepthMatrixList) > 5):
        prevDepthMatrixList.pop(0)

    obstacleList = []

    LINE_COLOR_RIGHT = [0,255,0]
    LINE_COLOR_LEFT = [255,0,0]
    HALF_FOV_DEGREES_VERT = 45
    CENTER_PIXEL_VERT = (leftImage.get_height() / 2)
    HALF_FOV_DEGREES_HORIZ = 55
    CENTER_PIXEL_HORIZ = (leftImage.get_width() / 2)

    OBSTACLE_THRESHOLD = 500
    PIXEL_REDUCTION_COEFFICIENT = 20

    # Use step distance of PIXEL_REDUCTION_COEFFICIENT to reduce the number of pixels that have to be checked
    for j in range(int(leftImage.get_height()/2), int(leftImage.get_height()), PIXEL_REDUCTION_COEFFICIENT):
        for i in range(0,leftImage.get_width(), PIXEL_REDUCTION_COEFFICIENT):
            if (lineImg[j,i,0] == LINE_COLOR_RIGHT[0] and lineImg[j,i,1] == LINE_COLOR_RIGHT[1] and lineImg[j,i,2] == LINE_COLOR_RIGHT[2]) or (lineImg[j,i,0] == LINE_COLOR_LEFT[0] and lineImg[j,i,1] == LINE_COLOR_LEFT[1] and lineImg[j,i,2] == LINE_COLOR_LEFT[2]):    #If the cooresponding image pixel is a line, then that depth value is an obstacle
                #Convert depth + pixel value into overhead x,y coordinates
                error, currentDepth = leftDepthMatrix.get_value(i, j) #currentDepth = getDepthMatrixValue(prevDepthMatrixList, i, j)
                #print("Found a line pixel")
                
                if not (math.isnan(currentDepth) or math.isinf(currentDepth)): # depth value is nan when the distance is too far away

                    phi_degrees = 1.000 * HALF_FOV_DEGREES_VERT * (j - CENTER_PIXEL_VERT)/CENTER_PIXEL_VERT
                    phi_radians =  phi_degrees * math.pi/180 # Angle to object in radians
                    correctedDepth = currentDepth * math.cos(phi_radians)

                    theta_degrees = 1.000 * HALF_FOV_DEGREES_HORIZ * (i - CENTER_PIXEL_HORIZ)/CENTER_PIXEL_HORIZ #Angle to object in degrees
                    theta_radians =  theta_degrees * math.pi/180 # Angle to object in radians

                    #Find overhead X coordinate
                    overheadXPos = correctedDepth * math.sin(theta_radians)

                    #Find overhead Y coordinate
                    overheadYPos = correctedDepth * math.cos(theta_radians)
                    #Add coordinate to obstacle list

                    if overheadYPos < OBSTACLE_THRESHOLD:
                        obstacleList.append((overheadXPos, overheadYPos))
                        #print("OBSTACLE FOUND AT X= " + str(overheadXPos) + " Y= " + str(overheadYPos))
                        pygame.draw.circle(window, (0,255,0), coordinateTransform(overheadXPos, overheadYPos), 2) # Green dots indicate lines
                

    centerY = int(leftImage.get_height() / 2)
    #for currentX in range(leftImage.get_width()):
    for currentX in range(0, leftImage.get_width(), 8):
        error, currentDepth = leftDepthMatrix.get_value(currentX, centerY)

        if not (math.isnan(currentDepth) or math.isinf(currentDepth)): # depth value is nan when the distance is too far away

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
                #print("OBSTACLE FOUND AT X= " + str(overheadXPos) + " Y= " + str(overheadYPos))
                obstacleList.append((overheadXPos, overheadYPos))
                pygame.draw.circle(window, (255,0,0), coordinateTransform(overheadXPos, overheadYPos), 2) # Red dots indicate obstacles


        
    #Return obstacle list
    return obstacleList

def comparePathToObstacles(coordinateList):
    ANGLE_INCREMENT = 1

    # TODO: This starting angle could be changed to work with compass navigation by 
    #       having the initial angle be the angle the compass wants and then navigate from there.
    #       This would only be implemented if compass navigation ends up being necessary
    angle = 0.0001

    minDistanceRight = 0
    minDistanceLeft = 0

    #print("Coord length: " + str(len(coordinateList)))

    
    while minDistanceRight < ROBOT_WIDTH/2 and minDistanceLeft < ROBOT_WIDTH/2 and angle < 55:
        angle_radians =  angle * math.pi/180 # Angle to object in radians
        slope = 1/math.tan(angle_radians)
        a = -1.000 * slope

        minDistanceRight = 5000 #10000
        for coord in coordinateList:
            distance = abs(a * coord[0] + coord[1])/math.sqrt(a*a + 1)
            if (distance < minDistanceRight):
                minDistanceRight = distance

        print("Min distance right: " + str(minDistanceRight))

        a = 1.000 * slope

        minDistanceLeft = 5000 #10000
        for coord in coordinateList:
            distance = abs(a * coord[0] + coord[1])/math.sqrt(a*a + 1)
            if (distance < minDistanceLeft):
                minDistanceLeft = distance
        
        angle += ANGLE_INCREMENT

    angle -= ANGLE_INCREMENT

    if (angle > 54): # if a path cant be found go forward
        return 0

    if (minDistanceRight > minDistanceLeft):
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

   # cv2.imshow("img", final)

    # Finds the overhead x,y coordinates of all obstacles and lines
    coordinateList = findCoordinatesOfObstacles(zed, lineImg)

    # Compare all obstacles points to current path
    desiredAngle = comparePathToObstacles(coordinateList)

    if (desiredAngle == 0): # If no path forward can be found, stop the robot
        return (0,0)

    # Constant representing the desired travel angle at which the inside wheel will stop while the outside wheel moves at double speed
    STOP_ANGLE = 55

    #Creates a differential between the wheel speeds proportional to the desired angle of travel
    leftSpeed = int(ROBOT_SPEED * (1 - (desiredAngle/STOP_ANGLE)))
    rightSpeed = int(ROBOT_SPEED * (1 + (desiredAngle/STOP_ANGLE)))

    end_pos_y = 400

    angle_radians =  desiredAngle * math.pi/180

    end_pos_x = end_pos_y * math.tan(angle_radians)
     
    pygame.draw.line(window, (0,0,255), coordinateTransform(0, 0), coordinateTransform(end_pos_x, end_pos_y), width=ROBOT_WIDTH)

    return (leftSpeed, rightSpeed)

def update_gps(gps:GPS):
    gps.updatePosition()
    coords = (gps.currentLocation["longitude"], gps.currentLocation["latitude"])
    cart = gps.get_diff(coords)
    return cart[0], cart[1]


def printGPSstat(gps:GPS):
    
    coords = (gps.currentLocation["latitude"], gps.currentLocation["longitude"])
    
    print("gps coords: ", gps.gps_to_ecef(coords))
    print("gps diff: ", gps.get_diff(coords))
    
    
def gpsMode(gps:GPS, motors:MotorController):
    
    # starting at 42.668016, -83.218338
    
    # write logic to turn toward waypoint, then move toward waypoint, navigating 
    # as we go
    
    
    # TODO turn left 90 degrees
    
    move(3, "left")
    
    x, y = update_gps(gps)
    speed = 20
    
    
    
    tp = 2.5
    tn = 1.2
    ct = time.time()
    arrived = lambda : True if (abs(x) < 3.0) and (abs(y) < 3.0) else False
    while (~arrived()):
        
        ct = sendMotorCommand(motors, "forward", ct)
        x,y = update_gps(gps)
        
        slope = -y / x
        while (slope > tp or ~arrived()):
            move(.3, "right")
            move(1, "forward")
            x, y = update_gps()
            
        while (slope < tn or ~arrived()):
            move(.3, "left")
            move(1, "forward")
            x, y = update_gps()


    motors.stop()
    

def move(seconds, direction:str):
    start = time.time()
    lastcommand = start
    while (time.time() - start < seconds):
        lastcommand = sendMotorCommand(motors, direction, lastcommand)

        



#----------------------------------------------------------------------
# Setting up usb connections
#----------------------------------------------------------------------
motors = MotorController('COM5')

BluetoothSerialObj = serial.Serial('COM4') # COMxx  format on Windows
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
# Setting up overhead view
#----------------------------------------------------------------------

# initiate pygame and give permission
# to use pygame's functionality.
pygame.init()
# create the display surface object
# of specific dimension.
window = pygame.display.set_mode((600, 600))
pygame.display.set_caption('Overhead View')
# Fill the screen with white color
window.fill((255, 255, 255))
# Draws the surface object to the screen.
pygame.display.update()

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

gps = GPS('COM8',  42.668016, -83.218338)

doGPS = False

#----------------------------------------------------------------------
# Main loop
#----------------------------------------------------------------------
while True:

    
    
    if (doGPS):
        while (gps.findWaypointDistance() is None):
            gps.updatePosition()
        gpsMode()
        break

    # Check for a bluetooth command
    if (BluetoothSerialObj.inWaiting() > 0):
        BluetoothString = BluetoothSerialObj.readline()
    else:
        BluetoothString = ""

    #TODO: Add watchdog timer

    # Always parse the bluetooth command even if the robot is in autonomous mode to look out for an stop command
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


    window.fill((255, 255, 255))
    pygame.draw.circle(window, (0,0,0), coordinateTransform(0, 0), 20) #Draw a black circle representing the robot in the overhead view
    #drawing grid lines
    pygame.draw.line(window, (0,0,0), coordinateTransform(0, 0), coordinateTransform(0, 600), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(100, 0), coordinateTransform(100, 600), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(200, 0), coordinateTransform(200, 600), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(-100, 0), coordinateTransform(-100, 600), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(-200, 0), coordinateTransform(-200, 600), width=2)

    #Add horizontal grid lines
    pygame.draw.line(window, (0,0,0), coordinateTransform(-300, 100), coordinateTransform(300, 100), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(-300, 200), coordinateTransform(300, 200), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(-300, 300), coordinateTransform(300, 300), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(-300, 400), coordinateTransform(300, 400), width=2)
    pygame.draw.line(window, (0,0,0), coordinateTransform(-300, 500), coordinateTransform(300, 500), width=2)


    # if the robot is in autonomous mode, set the current command to the best course of action according to the autonomous control logic
    if (doAutonomous):
        leftSpeed, rightSpeed = runAutonomousControls(zed)
        timeOfLastCommand = sendMotorCommand(motors, "custom", timeOfLastCommand, leftSpeed, rightSpeed)
    else:
        # send the next course of action to the motors based on manual controls
        timeOfLastCommand = sendMotorCommand(motors, currentCommand, timeOfLastCommand)

    pygame.display.update() #Update the overhead view


    #TODO: add logic to check if the robot is near the destination waypoint
    #       if the robot is near the waypoint, shut down
        
        

    # This is required, I don't know why but opencv doesn't work without this if statement contained within the main loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Shut down everything and close open ports
motors.shutDown()
BluetoothSerialObj.close()      # Close the port

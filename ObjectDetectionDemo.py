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
 
# initiate pygame and give permission
# to use pygame's functionality.
pygame.init()
 
# create the display surface object
# of specific dimension.
window = pygame.display.set_mode((600, 600))
 
# Fill the scree with white color
window.fill((255, 255, 255))
 
# Draws the surface object to the screen.
pygame.display.update()


#Defining Important Constants
DEPTH_VALUE = 300 # when objects are closer than DEPTH_VALUE, the robot will turn to avoid them
MAX_DEPTH = 400
ROBOT_SPEED = 20
ONE_SECOND_DELAY = 1000000000
ROBOT_WIDTH = 30 #Horizontal width of robot in cm
    

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
        shape = np.array([[int(0), int(y)], [int(x), int(y)], [int(x), int(0.6*y)], [int(0), int(0.6*y)]])

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
    
    return line_img


def linedetect(img):
    return hough_lines(img, 1, np.pi/180, 10, 20, 100).first

#hough_img = list(map(linedetect, canny_img))
#display_images(hough_img)

def processImage(image):
    #rgbImg = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)
    rgbImg = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)      #COLOR_BGRA2BGR

    interest = roi(image)
    
    filterimg = color_filter(interest)

    #ret,thresh1 = cv2.threshold(grayscale(filterimg),127,255,cv2.THRESH_BINARY)

    blur = cv2.GaussianBlur(grayscale(filterimg),(5,5),0)

    canny = cv2.Canny(blur, 50, 120)

    #function to combine all previous functions

    myline = hough_lines(canny, 1, np.pi/180, 10, 20, 5)

    weighted_img = cv2.addWeighted(myline, 1, rgbImg, 0.8, 0)
    
    cv2.imshow("img", weighted_img)


    ''' motorObj = MotorController('COM4')
    initialTime = time.time_ns()
    initialTime = sendMotorCommand(motorObj, command, initialTime)'''
    
    return weighted_img, myline


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

    OBSTACLE_THRESHOLD = 400
    PIXEL_REDUCTION_COEFFICIENT = 8

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
    for currentX in range(0, leftImage.get_width(), 4):
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

    angle = 0.0001

    minDistanceRight = 0
    minDistanceLeft = 0

    #print("Coord length: " + str(len(coordinateList)))

    
    while minDistanceRight < ROBOT_WIDTH/2 and minDistanceLeft < ROBOT_WIDTH/2 and angle < 55:
        angle_radians =  angle * math.pi/180 # Angle to object in radians
        slope = 1/math.tan(angle_radians)
        a = -1.000 * slope

        minDistanceRight = 10000
        for coord in coordinateList:
            distance = abs(a * coord[0] + coord[1])/math.sqrt(a*a + 1)
            if (distance < minDistanceRight):
                minDistanceRight = distance

        #print("Min distance right: " + str(minDistanceRight))

        a = 1.000 * slope

        minDistanceLeft = 10000
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

    #cv2.imshow("img", final)

    # Finds the overhead x,y coordinates of all obstacles and lines
    coordinateList = findCoordinatesOfObstacles(zed, lineImg)

    # Compare all obstacles points to current path
    desiredAngle = comparePathToObstacles(coordinateList)

    print("Turning " + str(int(desiredAngle)) + " degrees")

    end_pos_y = 400

    angle_radians =  desiredAngle * math.pi/180

    end_pos_x = end_pos_y * math.tan(angle_radians)
    
    pygame.draw.line(window, (0,0,255), coordinateTransform(0, 0), coordinateTransform(end_pos_x, end_pos_y), width=ROBOT_WIDTH)

    #desiredAngle = 0
    return desiredAngle


def coordinateTransform(xpos, ypos):
    transformedXpos = int(300 + xpos)
    transformedYpos = int(600 - ypos)

    return [transformedXpos, transformedYpos]

#----------------------------------------------------------------------
# Setting up usb connections
#----------------------------------------------------------------------
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
#----------------------------------------------------------------------
# Main loop
#----------------------------------------------------------------------
while True:
    # Fill the scree with white color
    window.fill((255, 255, 255))
    

    autonomousCommand = runAutonomousControls(zed)

    pygame.draw.circle(window, (0,0,255), coordinateTransform(0, 0), 20)

    # Draws the surface object to the screen.
    pygame.display.update()

    #print("Desired Angle:")
    #print(autonomousCommand)

    # if the robot is in autonomous mode, set the current command to the best course of action according to the autonomous control logic
    #if (doAutonomous):
    #    currentCommand = autonomousCommand
        

    # send the next course of action to the motors
    #timeOfLastCommand = sendMotorCommand(motors, currentCommand, timeOfLastCommand)

    # This is required, I don't know why but opencv doesn't work without this if statement contained within the main loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



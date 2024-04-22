
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


class Control():
        
        #Defining Important Constants
    DEPTH_VALUE = 300 # when objects are closer than DEPTH_VALUE, the robot will turn to avoid them
    MAX_DEPTH = 400
    ROBOT_SPEED = 20
    ONE_SECOND_DELAY = 1000000000
    ROBOT_WIDTH = 100 #Horizontal width of robot in cm

    WAYPOINT_LATITUDE = 37.228889
    WAYPOINT_LONGITUDE = -80.4155556
    
    
    def __init__(self, zed):
        self.zed = zed
        self.rightSlope, self.leftSlope, self.rightIntercept, self.leftIntercept = [],[],[],[]
        self.prevDepthMatrixList = []

    def color_filter(self, image):
        #convert from RGB to HLS (Hue, Lightness, Saturation)
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        lower = np.array([0,190,0])
        upper = np.array([255,255,255])

        whitemask = cv2.inRange(hls, lower, upper)
        
        masked = cv2.bitwise_and(image, image, mask = whitemask)    
        
        return masked

    @staticmethod
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

    @staticmethod
    def grayscale(img):
        #canny needs a gray image, so we convert
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    @staticmethod
    def canny(img):
        #using canny to get edges
        return cv2.Canny(Control.grayscale(img), 50, 120)


    
   
    
    
    def draw_lines(self, img, lines):
        global prevRightSlope, prevLeftSlope
       
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
                            self.rightSlope.append(slope)
                            self.rightIntercept.append(yintercept)
                        else: None                
                    elif slope < -0.3:
                        if x1 < 600:
                            yintercept = y2 - (slope*x2)                    
                            self.leftSlope.append(slope)
                            self.leftIntercept.append(yintercept)    
        else:
            hello = 0
                        
                        
        #We use slicing operators and np.mean() to find the averages of the 30 previous frames
        #This makes the lines more stable, and less likely to glitch
        leftavgSlope = np.mean(self.leftSlope[-30:])
        leftavgIntercept = np.mean(self.leftIntercept[-30:])
        
        rightavgSlope = np.mean(self.rightSlope[-30:])
        rightavgIntercept = np.mean(self.rightIntercept[-30:])
        
        #plotting the lines
        
        left_line_x1 = None 
        right_line_x1 = None
        #plotting the lines
        try:
            #if right line does not exist
            if math.isnan(rightavgSlope):
        #draw left line and everything to the right
                left_line_x1 = int((0.65*img.shape[0] - leftavgIntercept)/leftavgSlope)
                left_line_x2 = int((img.shape[0] - leftavgIntercept)/leftavgSlope)

                pts = np.array([[left_line_x1, int(0.65*img.shape[0])],[left_line_x2, int(img.shape[0])],[int(img.shape[1]), int(img.shape[0])],[int(img.shape[1]), int(0.5*img.shape[1])]], np.int32)
                pts = pts.reshape((-1,1,2))
                cv2.fillPoly(img,[pts],(0,0,255))  

                cv2.line(img, (left_line_x1, int(0.65*img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
                return [left_line_x1, float("nan"), leftavgSlope, float("nan")]

            #if left line does not exist
            if math.isnan(leftavgSlope):
            #draw right line and everything to the left
                right_line_x1 = int((0.65*img.shape[0] - rightavgIntercept)/rightavgSlope)
                right_line_x2 = int((img.shape[0] - rightavgIntercept)/rightavgSlope)

                pts = np.array([[15, int(0.65*img.shape[0])],[0, int(img.shape[0])],[right_line_x2, int(img.shape[0])],[right_line_x1, int(0.65*img.shape[0])]], np.int32)
                pts = pts.reshape((-1,1,2))
                cv2.fillPoly(img,[pts],(0,0,255)) 

                cv2.line(img, (right_line_x1, int(0.65*img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)
                return [float("nan"), right_line_x1, float("nan"), rightavgSlope]
            
            #both lines exist
            left_line_x1 = int((0.65*img.shape[0] - leftavgIntercept)/leftavgSlope)
            left_line_x2 = int((img.shape[0] - leftavgIntercept)/leftavgSlope)
        
            right_line_x1 = int((0.65*img.shape[0] - rightavgIntercept)/rightavgSlope)
            right_line_x2 = int((img.shape[0] - rightavgIntercept)/rightavgSlope)

            pts = np.array([[left_line_x1, int(0.65*img.shape[0])],[left_line_x2, int(img.shape[0])],[right_line_x2, int(img.shape[0])],[right_line_x1, int(0.65*img.shape[0])]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.fillPoly(img,[pts],(0,0,255))      
            
            
            cv2.line(img, (left_line_x1, int(0.65*img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 20)
            cv2.line(img, (right_line_x1, int(0.65*img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 20)
        except ValueError:
                #I keep getting errors for some reason, so I put this here. Idk if the error still persists.
            pass

        return [left_line_x1, right_line_x1]
        
        
                    
    def hough_lines(self,img, rho, theta, threshold, min_line_len, max_line_gap):
        """
        `img` should be the output of a Canny transform.
        """
        #using hough to get the lines from the canny image
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        coors = self.draw_lines(line_img, lines)
        
        return line_img


    

    def processImage(self,image):
        #rgbImg = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)
        rgbImg = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)      #COLOR_BGRA2BGR

        #function to combine all previous functions
        interest = Control.roi(image)
        
        filterimg = Control.color_filter(interest)

        blur = cv2.GaussianBlur(Control.grayscale(filterimg), (5,5),0)

        canny = cv2.Canny(blur, 50, 120)

        myline = self.hough_lines(canny, 1, np.pi/180, 10, 20, 5)

        weighted_img = cv2.addWeighted(myline, 1, rgbImg, 0.8, 0)
        
        #cv2.imshow("img", weighted_img)
        
        return weighted_img, myline

    @staticmethod
    def coordinateTransform(xpos, ypos):
        transformedXpos = int(300 + xpos)
        transformedYpos = int(600 - ypos)

        return [transformedXpos, transformedYpos]

    @staticmethod
    def getDepthMatrixValue(depthList, xVal, yVal):
        
        average = 0
        for matrix in depthList:
            average += matrix.get_value(xVal, yVal)

        average = average/len(depthList)

        return average

    

    def findCoordinatesOfObstacles(self, lineImg):
        

        leftImage = sl.Mat()
        leftDepthMatrix = sl.Mat()
        runtime_params = sl.RuntimeParameters()
        error = self.zed.grab(runtime_params)
        self.zed.retrieve_image(leftImage, sl.VIEW.LEFT)
        self.zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image

        # Add the current depth matrix to the list of previous depth matrix measurements
        # Enables depth value measurements to be averaged over the last couple of frames
        self.prevDepthMatrixList.append(leftDepthMatrix)

        if (len(self.prevDepthMatrixList) > 5):
            self.prevDepthMatrixList.pop(0)

        obstacleList = []

        LINE_COLOR_RIGHT = [0,255,0]
        LINE_COLOR_LEFT = [255,0,0]
        HALF_FOV_DEGREES_VERT = 45
        CENTER_PIXEL_VERT = (leftImage.get_height() / 2)
        HALF_FOV_DEGREES_HORIZ = 55
        CENTER_PIXEL_HORIZ = (leftImage.get_width() / 2)

        OBSTACLE_THRESHOLD = 400
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
                            pygame.draw.circle(self.window, (0,255,0), Control.coordinateTransform(overheadXPos, overheadYPos), 2) # Green dots indicate lines
                    

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
                    pygame.draw.circle(self.window, (255,0,0), Control.coordinateTransform(overheadXPos, overheadYPos), 2) # Red dots indicate obstacles


            
        #Return obstacle list
        return obstacleList

    
    def comparePathToObstacles(self, coordinateList):
        ANGLE_INCREMENT = 1

        # TODO: This starting angle could be changed to work with compass navigation by 
        #       having the initial angle be the angle the compass wants and then navigate from there.
        #       This would only be implemented if compass navigation ends up being necessary
        angle = 0.0001

        minDistanceRight = 0
        minDistanceLeft = 0

        #print("Coord length: " + str(len(coordinateList)))

        
        while minDistanceRight < self.ROBOT_WIDTH/2 and minDistanceLeft < self.ROBOT_WIDTH/2 and angle < 55:
            angle_radians =  angle * math.pi/180 # Angle to object in radians
            slope = 1/math.tan(angle_radians)
            a = -1.000 * slope

            minDistanceRight = 10000
            for coord in coordinateList:
                distance = abs(a * coord[0] + coord[1])/math.sqrt(a*a + 1)
                if (distance < minDistanceRight):
                    minDistanceRight = distance

            print("Min distance right: " + str(minDistanceRight))

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
    def runAutonomousControls(self, zed):
        #depthSensorDirection = runDepthSensor(zed) # Finds the best course of action according to the depth sensor

        # Get the image from the zed camera
        leftImage1 = sl.Mat()
        runtime_params = sl.RuntimeParameters()
        error = zed.grab(runtime_params)
        zed.retrieve_image(leftImage1, sl.VIEW.LEFT)

        frame = leftImage1.get_data()
        final, lineImg = self.processImage(frame)  # process the current image and color the lines in a solid color

        cv2.imshow("img", final)

        # Finds the overhead x,y coordinates of all obstacles and lines
        coordinateList = self.findCoordinatesOfObstacles(zed, lineImg)

        # Compare all obstacles points to current path
        desiredAngle = self.comparePathToObstacles(coordinateList)

        if (desiredAngle == 0): # If no path forward can be found, stop the robot
            return (0,0)

        # Constant representing the desired travel angle at which the inside wheel will stop while the outside wheel moves at double speed
        STOP_ANGLE = 55

        #Creates a differential between the wheel speeds proportional to the desired angle of travel
        leftSpeed = int(self.ROBOT_SPEED * (1 - (desiredAngle/STOP_ANGLE)))
        rightSpeed = int(self.ROBOT_SPEED * (1 + (desiredAngle/STOP_ANGLE)))

        return (leftSpeed, rightSpeed)
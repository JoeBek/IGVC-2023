import pyzed.sl as sl
import math


class Path():

    def __init__(self):
        self.prevDepthMatrix = []
        self.lineList = []
        self.obstacleList = []
        self.allObstacles = []
        self.robot_width = 100
        self.angle = 0
        self.angle_increment = 1

    def coordinateTransform(x,y):
        transformedXpos = int(300 + x)
        transformedYpos = int(600 - y)

        return [transformedXpos, transformedYpos]
    
    # Function is not used
    def getDepthMatrixValue(depthList, xVal, yVal):
        average = 0
        for matrix in depthList:
            average += matrix.get_value(xVal, yVal)

        average = average/len(depthList)

        return average
    
    # Possibly seperate this function into obstacle and line detections
    def findCoordinatesOfObstacles(self, image, zed):
        leftImage = sl.Mat()
        leftDepthMatrix = sl.Mat()
        runtime_params = sl.RuntimeParameters()
        error = zed.grab(runtime_params)
        zed.retrieve_image(leftImage, sl.VIEW.LEFT)
        zed.retrieve_measure(leftDepthMatrix, sl.MEASURE.DEPTH)  # gets left depth image

        self.prevDepthMatrixList.append(leftDepthMatrix)

        if (len(self.prevDepthMatrixList) > 5):
            self.prevDepthMatrixList.pop(0)

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
                if (image[j,i,0] == LINE_COLOR_RIGHT[0] and image[j,i,1] == LINE_COLOR_RIGHT[1] and image[j,i,2] == LINE_COLOR_RIGHT[2]) or (image[j,i,0] == LINE_COLOR_LEFT[0] and image[j,i,1] == LINE_COLOR_LEFT[1] and image[j,i,2] == LINE_COLOR_LEFT[2]):    #If the cooresponding image pixel is a line, then that depth value is an obstacle
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
                            self.lineList.append((overheadXPos, overheadYPos))
                            #print("OBSTACLE FOUND AT X= " + str(overheadXPos) + " Y= " + str(overheadYPos))
                            #pygame.draw.circle(self.window, (0,255,0), self.coordinateTransform(overheadXPos, overheadYPos), 2) # Green dots indicate lines

        self.allObstacles.append(self.lineList)

        centerY = int(CENTER_PIXEL_VERT)
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
                    self.obstacleList.append((overheadXPos, overheadYPos))
                    #pygame.draw.circle(window, (255,0,0), coordinateTransform(overheadXPos, overheadYPos), 2) # Red dots indicate obstacles

        self.allObstacles.append(self.obstacleList)

    def comparePathToObstacles(self):
        self.angle = 0.0001

        minDistanceRight = 0
        minDistanceLeft = 0

        while minDistanceRight < self.robot_width/2 and minDistanceLeft < self.robot_width/2 and self.angle < 55:
            angle_radians =  self.angle * math.pi/180 # Angle to object in radians
            slope = 1/math.tan(angle_radians)
            a = -1.000 * slope

            minDistanceRight = float("inf") #10000
            for coord in self.allObstacles:
                distance = abs(a * coord[0] + coord[1])/math.sqrt(a*a + 1)
                if (distance < minDistanceRight):
                    minDistanceRight = distance

            print("Min distance right: " + str(minDistanceRight))

            a = 1.000 * slope

            minDistanceLeft = float("inf") #10000
            for coord in self.allObstacles:
                distance = abs(a * coord[0] + coord[1])/math.sqrt(a*a + 1)
                if (distance < minDistanceLeft):
                    minDistanceLeft = distance
            
            self.angle += self.angle_increment

        self.angle -= self.angle_increment

        if (self.angle > 54): # if a path cant be found go forward
            self.angle = 0
            return self.angle

        if (minDistanceRight > minDistanceLeft):
            return self.angle
        
        
        return -1 * self.angle
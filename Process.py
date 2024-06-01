import cv2
import numpy as np
import math

class Process():

    def __init__(self, image):
        self.rightSlope = []
        self.leftSlope = []
        self.rightIntercept = []
        self.leftIntercept = []
        self.prevRightSlope = 0
        self.prevLeftSlope = 0

        self.image = image 
        self.rgbImg = None
        self.interestImg = None
        self.filterImg = None
        self.grayImg = None
        self.blurImg = None
        self.cannyImg = None
        self.houghImg = None
        self.weightImg = None
    
    def rgb(self):
        self.rgbImg = cv2.cvtColor(self.image, cv2.COLOR_BGRA2BGR)

    def roi(self):
        if self.image is not None:
            x = int(self.image.shape[1])
            y = int(self.image.shape[0])
            shape = np.array([[int(0), int(y-100)], [int(x), int(y-100)], [int(0.55*x), int(0.6*y)], [int(0.45*x), int(0.6*y)]])

            mask = np.zeros_like(self.image)

        #Uses 3 channels or 1 channel for color depending on input imageq
            if len(self.image.shape) > 2:
                channel_count = self.image.shape[2]
                ignore_mask_color = (255,) * channel_count
            else:
                ignore_mask_color = 255

        #creates a polygon with the mask color (for area between lines)
            cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)

        #returns the image only where the mask pixels are not zero
            masked_image = cv2.bitwise_and(self.image, mask)
            self.interestImg = masked_image
        else: 
            print("None")

    def color_filtering(self, interest):
        hls = cv2.cvtColor(interest, cv2.COLOR_RGB2HLS)
        lower = np.array([0,190,0])
        upper = np.array([255,255,255])

        whitemask = cv2.inRange(hls, lower, upper)
    
        masked = cv2.bitwise_and(interest, interest, mask = whitemask)    
    
        self.filterImg = masked

    def grayscale(self, filter):
        self.grayImg = cv2.cvtColor(filter, cv2.COLOR_RGB2GRAY)

    def blur(self, grayImg):
        self.blurImg = cv2.GaussianBlur(grayImg, (5,5),0)

    def canny(self, blurTerm):
        self.cannyImg = cv2.Canny(blurTerm, 100, 150)

    def draw_lines(self, lineImg, lines):

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
            return [None, None]
                    
                    
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
            if math.isnan(rightavgSlope):
            #draw left line and everything to the right
                left_line_x1 = int((0.65*lineImg.shape[0] - leftavgIntercept)/leftavgSlope)
                left_line_x2 = int((lineImg.shape[0] - leftavgIntercept)/leftavgSlope)

                pts = np.array([[left_line_x1, int(0.65*lineImg.shape[0])],[left_line_x2, int(lineImg.shape[0])],[int(lineImg.shape[1]), int(lineImg.shape[0])],[int(lineImg.shape[1]), int(0.5*lineImg.shape[1])]], np.int32)
                pts = pts.reshape((-1,1,2))
                cv2.fillPoly(lineImg,[pts],(0,0,255))  

                cv2.line(lineImg, (left_line_x1, int(0.65*lineImg.shape[0])), (left_line_x2, int(lineImg.shape[0])), leftColor, 10)
                return [left_line_x1, float("nan"), leftavgSlope, float("nan")]

            if math.isnan(leftavgSlope):
            #draw right line and everything to the left
                right_line_x1 = int((0.65*lineImg.shape[0] - rightavgIntercept)/rightavgSlope)
                right_line_x2 = int((lineImg.shape[0] - rightavgIntercept)/rightavgSlope)

                pts = np.array([[15, int(0.65*lineImg.shape[0])],[0, int(lineImg.shape[0])],[right_line_x2, int(lineImg.shape[0])],[right_line_x1, int(0.65*lineImg.shape[0])]], np.int32)
                pts = pts.reshape((-1,1,2))
                cv2.fillPoly(lineImg,[pts],(0,0,255)) 

                cv2.line(lineImg, (right_line_x1, int(0.65*lineImg.shape[0])), (right_line_x2, int(lineImg.shape[0])), rightColor, 10)
                return [float("nan"), right_line_x1, float("nan"), rightavgSlope]
            
            left_line_x1 = int((0.65*lineImg.shape[0] - leftavgIntercept)/leftavgSlope)
            left_line_x2 = int((lineImg.shape[0] - leftavgIntercept)/leftavgSlope)
    
            right_line_x1 = int((0.65*lineImg.shape[0] - rightavgIntercept)/rightavgSlope)
            right_line_x2 = int((lineImg.shape[0] - rightavgIntercept)/rightavgSlope)

            pts = np.array([[left_line_x1, int(0.65*lineImg.shape[0])],[left_line_x2, int(lineImg.shape[0])],[right_line_x2, int(lineImg.shape[0])],[right_line_x1, int(0.65*lineImg.shape[0])]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.fillPoly(lineImg,[pts],(0,0,255))      
        
        
            cv2.line(lineImg, (left_line_x1, int(0.65*lineImg.shape[0])), (left_line_x2, int(lineImg.shape[0])), leftColor, 20)
            cv2.line(lineImg, (right_line_x1, int(0.65*lineImg.shape[0])), (right_line_x2, int(lineImg.shape[0])), rightColor, 20)
        except ValueError:
                #I keep getting errors for some reason, so I put this here. Idk if the error still persists.
            pass

        return [left_line_x1, right_line_x1]

    def hough_lines(self, cannyTerm, rho, theta, threshold, min_line_len, max_line_gap):
        """
        `img` should be the output of a Canny transform.
        """
        #using hough to get the lines from the canny image
        lines = cv2.HoughLinesP(cannyTerm, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((cannyTerm.shape[0], cannyTerm.shape[1], 3), dtype=np.uint8)
        coors = self.draw_lines(line_img, lines)

        #line_img2 = cv2.circle(line_img, (coors[0], 400), radius=10, color=(255, 255, 255), thickness=-6)
        #line_img2 = cv2.circle(line_img2, (coors[1], 400), radius=10, color=(255, 255, 255), thickness=-6)      
    
        self.houghImg = line_img

    def weight(self, houghTerm, rgbTerm):
        self.weightImg = cv2.addWeighted(houghTerm, 1, rgbTerm, 0.8, 0)

    def process(self):
        self.rgb()
        self.roi()
        self.color_filtering(self.interest)
        self.grayscale(self.filter)
        self.blur(self.gray)
        self.canny(self.blurImg)
        self.hough_lines(self.cannyImg)
        self.weight(self.houghImg, self.rgbImg)
        return self.houghImg, self.weightImg


    

    
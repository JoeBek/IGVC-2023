import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math
import cv2
import time
# %matplotlib inline
# from moviepy.editor import VideoFileClip
# from IPython.display import HTML
from MotorControlAPI import MotorController

ROBOT_SPEED = 20
ONE_SECOND_DELAY = 1000000000


# myVar = MotorController('COMx')


# print(imageFiles)

def display_images(images, cmap=None):
    plt.figure(figsize=(40, 40))
    for i, image in enumerate(images):
        plt.subplot(3, 2, i + 1)
        plt.imshow(image, cmap)
        plt.autoscale(tight=True)
    plt.show()


# display_images(imageList)


def color_filter(image):
    # convert from RGB to HLS (Hue, Lightness, Saturation)
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    lower = np.array([0, 190, 0])
    upper = np.array([255, 255, 255])

    whitemask = cv2.inRange(hls, lower, upper)

    masked = cv2.bitwise_and(image, image, mask=whitemask)

    return masked


# filtered_img = list(map(color_filter, imageList))

# display_images(filtered_img)


def roi(img):
    # function to idenify region of interest, using a triangle to focus on where the lines are
    if img is not None:
        # get coords of actual frame of gui (get gui dimensions)
        x = int(img.shape[1]) # seems to be 600
        # print("x = ", x)
        y = int(img.shape[0]) # seems to be 480
        # print("y = ", y)

        # oh thats crazy this is the roi shape dimensions
        """
        shape = np.array(
            [[int(0), int(y)], [int(x), int(y)], [int(0.55 * x), int(0.6 * y)], [int(0.45 * x), int(0.6 * y)]])
        """
        """
        print(shape)
        [[  0 480]
         [600 480]
         [330 288]
         [270 288]]
        """

        shape = np.array(
            [[int(0), int(y)], [int(x), int(y)], [int(x), int(0.5 * y)], [int(0), int(0.5 * y)]])
        """
        # lets make it a rect
        [[  0 480]  bottom left
         [600 480]  bottom right
         [600 240]    top right
         [ 0 240]]    top left
        """
        mask = np.zeros_like(img)
        # print(img)   # returns rgb values (0-255) of each pixel of frame
        # print(mask)  # all zeros, i assume it is the same size array as img

        # Uses 3 channels or 1 channel for color depending on input image
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255


        # print(np.int32([shape])) = shape lol
        # creates a polygon with the mask color (for area between lines)
        cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)
        # cv2.imshow("uh does this work", mask)  # should return the roi of the frame

        # returns the image only where the mask pixels are not zero
        masked_image = cv2.bitwise_and(img, mask)

        return masked_image
    else:
        print("None")

# roi_img = list(map(roi, filtered_img))
# display_images(roi_img)

def grayscale(img):
    # canny needs a gray image, so we convert
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


def canny(img):
    # using canny to get edges
    return cv2.Canny(grayscale(img), 50, 120)


# canny_img = list(map(canny, roi_img))
# display_images(canny_img,cmap='gray')


def draw_lines(img, lines, thickness=5):
    store_line_x1, store_line_x2, store_line_y1, store_line_y2 = [], [], [], []

    print("START OF DRAW LINES")
    rightColor = [0, 255, 0]
    leftColor = [255, 0, 0]

    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    if lines is not None:
        for line in lines: # [260 304 261 328]
            for x1, y1, x2, y2 in line:
                # for perpendicular line, slope = 0
                # and should be 1 line, not two
                # slope = (y1 - y2) / (x1 - x2)
                print(x1, x2, y1, y2)
                if ((y1 - y2) < 20): # chose 10 for now for line thickness
                    if y1 > 240:
                        store_line_x1.append(x1)
                        store_line_x2.append(x2)
                        store_line_y1.append(y1)
                        store_line_y2.append(y2)
                    else:
                        None
    else:
        # print("was None")
        hello = 0

    # We use slicing operators and np.mean() to find the averages of the 30 previous frames
    # This makes the lines more stable, and less likely to glitch
    linex1avg = np.mean(store_line_x1[-30:])
    linex2avg = np.mean(store_line_x2[-30:])
    liney1avg = np.mean(store_line_y1[-30:])
    liney2avg = np.mean(store_line_y2[-30:])

    line_x1 = None
    # plotting the lines
    #try:
    if np.isnan(linex1avg):
        print("hello")
    else:
    # if linex1avg is not None:
        line_x1 = int((0.65 * img.shape[0]))
        line_x2 = int((img.shape[0]))

        #"""
        print("linex1avg = ", (linex1avg))
        print("linex2avg = ", linex2avg)
        print("liney1avg = ", liney1avg)
        print("liney2avg = ", liney2avg)
        #"""

        #print("imgshape0 = ", img.shape[0]) # y = 480
        #print("imgshape1 = ", img.shape[1]) # x = 600

        #start_coord = int(linex1avg), int(liney1avg) # X,Y
        #end_coord = int(linex2avg), int(liney2avg)

        # maybe dont use x avergae? just use x frame info?
        start_coord = int(0), int(liney1avg) # X,Y
        end_coord = int(img.shape[1]), int(liney2avg)

        print("sc = ", start_coord)
        print("ec = ", end_coord)

        cv2.line(img, start_coord, end_coord, leftColor, 10)

    #except ValueError:
        # I keep getting errors for some reason, so I put this here. Idk if the error still persists.
        #pass

    print('wait can we return this. linex1 = ', line_x1)
    line_x1 = int((0.65 * img.shape[0]))
    return [line_x1]


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
   #hough_lines(canny, 1, np.pi / 180, 10, 20, 5)
    """
    `img` should be the output of a Canny transform.
    """
    # using hough to get the lines from the canny image
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    coors = draw_lines(line_img, lines)
    # line_img2 = cv2.circle(line_img, (coors[0], 400), radius=10, color=(255, 255, 255), thickness=-6)
    # line_img2 = cv2.circle(line_img2, (coors[1], 400), radius=10, color=(255, 255, 255), thickness=-6)
    direction = "up"
    print("coors = ", coors)

    if coors is not None:
        left_diff = int(coors[0])
        # right_diff = 635 - int(coors[1])
        padding = 20
        print("left: ", left_diff)
        # print("right: ", right_diff)
        # r = 400
        """
        if (right_diff - padding < left_diff) and (left_diff < right_diff + padding):
            print("stay straight")
            direction = "up"
        elif (left_diff < right_diff - padding):
            print("move left")
            direction = "left"
        elif (left_diff > right_diff + padding):
            print("move right")
            direction = "right"
        """


        '''if (left_diff < right_diff - padding) or (left_diff > right_diff + padding): 
            print("move right")
            #myVar.turnRight(20)
        elif (right_diff < left_diff - padding) or (right_diff > left_diff + padding): 
            print("move left")
            #myVar.turnLeft(20)
        elif left_diff == right_diff: 
            print("stay straight")
            #myVar.forward(20)'''

    #motorObj = MotorController('COM4')
    #initialTime = time.time_ns()
    #initialTime = sendMotorCommand(motorObj, direction, initialTime)

    return line_img


#def linedetect(img):
    #return h#ough_lines(img, 1, np.pi / 180, 10, 20, 100).first


# hough_img = list(map(linedetect, canny_img))
# display_images(hough_img)

def processImage(image):
    # function to combine all previous functions
    interest = roi(image)

    filterimg = color_filter(interest)
    grayimg = grayscale(filterimg)
    canny = cv2.Canny(grayimg, 50, 120)
    myline = hough_lines(canny, 1, np.pi / 180, 10, 20, 5)

    weighted_img = cv2.addWeighted(myline, 1, image, 0.8, 0)

    cv2.imshow("img", interest)  # should return the roi of the frame
    # cv2.imshow("img", filterimg)  # very scary HLS image
    # cv2.imshow("img", grayimg)  # it is gray now
    # cv2.imshow("img", canny)  # canny is wack
    # cv2.imshow("img", myline)  # myline is wack

    cv2.imshow("hello", weighted_img)

    ''' motorObj = MotorController('COM4')
    initialTime = time.time_ns()
    initialTime = sendMotorCommand(motorObj, command, initialTime)'''

    # return weighted_img # uncomment if need to make final have a value in while loop


def sendMotorCommand(motorObj, command, lastCommandTime):
    if (time.time_ns() > lastCommandTime + (ONE_SECOND_DELAY * 0.1)):
        lastCommandTime = time.time_ns()
        if (command == "up"):
            print("Going Forward")
            motorObj.forward(ROBOT_SPEED)
        elif (command == "down"):
            print("Going Backwards")
            motorObj.backward(ROBOT_SPEED)
        elif (command == "left"):
            print("Going Left")
            motorObj.turnLeft(ROBOT_SPEED * 2)
        elif (command == "right"):
            print("Going Right")
            motorObj.turnRight(ROBOT_SPEED * 2)
        elif (command == "stop"):
            print("Stopping")
            motorObj.stop()
    return lastCommandTime


cap = cv2.VideoCapture(0)  # use cv2.VideoCapture(0) to access camera
while (cap.isOpened()):
    _, frame = cap.read()
    frame2 = frame[0:500, 0:600]
    # final = processImage(frame2)

    # call processImage
    processImage(frame2)

    # print("hello")
    # cv2.imshow("final", final)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
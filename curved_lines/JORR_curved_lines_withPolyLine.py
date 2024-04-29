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
        shape = np.array(
            [[int(0), int(y)], [int(x), int(y)], [int(x), int(0.4 * y)], [int(0), int(0.4 * y)]])

        """
        print(shape)
        [[  0 480]
         [600 480]
         [330 288]
         [270 288]]
        """

        # shape = np.array(
            # [[int(0.4 * x), int(y)], [int(x), int(y)], [int(x), int(0.5 * y)], [int(0.4 * x), int(0.8 * y)]])

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

rightSlope, leftSlope, rightIntercept, leftIntercept = [], [], [], []
storeLeftCords, storeRightCords = [], []

def draw_lines(img, lines, thickness=5):
    global rightSlope, leftSlope, rightIntercept, leftIntercept
    global storeLeftCords, storeRightCords

    rightColor = [0, 0, 255]
    leftColor = [255, 0, 0]

    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b

    # If curve to right, then slope negative for both lines?
    minX = 0 # basically -inf
    if lines is not None:
        print("START---")
        for line in lines: # [260 304 261 328]
            for x1, y1, x2, y2 in line:
                # print(x1, x2, y1, y2)
                slope = (y1 - y2) / (x1 - x2) # -24 / -1 = 24
                # print("slope: ", slope)

                # code will find ALL lines, no matter slope
                # get rid of horizontal lines
                if (slope < -0.1): # maybe 0.4 idk
                    # what if we store the x/y values in array?
                    if x1 > 300: # assume right i guess
                        print("slope", slope)
                        print("X1: ", x1)
                        print("Y1: ", y1)
                        print("X2: ", x2)
                        print("Y2: ", y2)
                        print(minX)
                        if x1 > minX:
                            storeRightCords.append([x1, y1])
                            storeRightCords.append([x2, y2])
                            minX = x1
                    elif x1 < 200:
                        print("slope", slope)
                        print("X1: ", x1)
                        print("Y1: ", y1)
                        print("X2: ", x2)
                        print("Y2: ", y2)
                        storeLeftCords.append([x1, y1])
                        storeLeftCords.append([x2, y2])

                else:
                    print("um", slope)

                """
                                if slope < -0.2:
                    if x1 > 300:
                        yintercept = y2 - (slope * x2)
                        rightSlope.append(slope)
                        rightIntercept.append(yintercept)
                    elif x1 < 600:
                        yintercept = y2 - (slope * x2)
                        leftSlope.append(slope)
                        leftIntercept.append(yintercept)
                    else:
                        None
                """
                """
                                elif slope > 0.2:
                    if x1 < 600:
                        yintercept = y2 - (slope * x2)
                        leftSlope.append(slope)
                        leftIntercept.append(yintercept)
                        # print("works")
                    elif x1 > 500:
                        yintercept = y2 - (slope * x2)
                        rightSlope.append(slope)
                        rightIntercept.append(yintercept)
                """
    else:
        # print("was None")
        hello = 0

    # print("end of draw_lines")

    # We use slicing operators and np.mean() to find the averages of the 30 previous frames
    # This makes the lines more stable, and less likely to glitch
    leftavgSlope = np.mean(leftSlope[-30:])
    leftavgIntercept = np.mean(leftIntercept[-30:])

    rightavgSlope = np.mean(rightSlope[-30:])
    rightavgIntercept = np.mean(rightIntercept[-30:])

    """
    print("leftavgSlope = ", (leftavgSlope))
    print("leftavgIntercept = ", leftavgIntercept)
    print("rightavgSlope = ", rightavgSlope)
    print("rightavgIntercept = ", rightavgIntercept)
    """

    left_line_x1 = None
    right_line_x1 = None

    recentLeftCords = storeLeftCords[-10:]
    recentRightCords = storeRightCords[-10:]
    print(recentLeftCords)
    print(recentRightCords)

    recentLeftCords.sort() # organized smallest to largest values
    print("recentLeftCords = ", recentLeftCords)
    popCords = recentLeftCords.pop(0) # get smallest coords
    print("popCords = ", popCords)


    leftPts = np.array(recentLeftCords, np.int32)
    rightPts = np.array(recentRightCords, np.int32)
    # print(leftPts)
    # print(rightPts)

    print(storeLeftCords)
    print(storeRightCords)

    storeLeftCords.clear()
    storeRightCords.clear()

    isClosed = True
    color = (255, 0, 0)
    thickness = 2
    cv2.polylines(img, [leftPts], isClosed, color, thickness)

    colorz = (0, 255, 0)
    cv2.polylines(img, [rightPts], isClosed, colorz, thickness)

    # compare
    # cv2.line(img, (359, 166), (381, 154), rightColor, 10)
    # cv2.line(img, (382, 153), (408, 142), rightColor, 10)


    # plotting the lines
    try:
        left_line_x1 = int((0.65 * img.shape[0] - leftavgIntercept) / leftavgSlope)
        # left_line_x2 = int((img.shape[0] - leftavgIntercept) / leftavgSlope)

        right_line_x1 = int((0.65 * img.shape[0] - rightavgIntercept) / rightavgSlope)
        # right_line_x2 = int((img.shape[0] - rightavgIntercept) / rightavgSlope)

        """
        print('left_line_x1 = ', left_line_x1)
        print('left_line_x2 = ', left_line_x2)
        print('right_line_x1 = ', right_line_x1)
        print('right_line_x2 = ', right_line_x2)
        """

        # pts = np.array([[left_line_x1, int(0.65 * img.shape[0])], [left_line_x2, int(img.shape[0])],
                        # [right_line_x2, int(img.shape[0])], [right_line_x1, int(0.65 * img.shape[0])]], np.int32)

        """
        ptsX = np.array([left_line_x1, int(0.65 * img.shape[0]), left_line_x2, int(img.shape[0])])
        # ERROR: ptsY = np.array([right_line_x2, int(img.shape[0])], [right_line_x1, int(0.65 * img.shape[0])])
        ptsY = np.array([right_line_x2, int(img.shape[0]), right_line_x1, int(0.65 * img.shape[0])])

        z = np.polyfit(ptsX, ptsY, 2)
        print("Z = ", z)
        lspace = np.linspace(0, 1000, 100)
        draw_x = lspace
        draw_y = np.polyval(z, draw_x)  # evaluate the polynomial
        draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32)  # needs to be int32 and transposed
        #print("DRAW LINES = ", draw_points)
        cv2.polylines(img, [draw_points], False, (255, 0, 0), 2)
        print("array of points = ", ptsX)
        print("array of points = ", ptsY)
        """

        # print("array of points = ", pts)
        # pts = pts.reshape((-1, 1, 2))
        # print("after = ", pts)
        # cv2.fillPoly(img, [pts], (0, 0, 255))
        # cv2.polylines(img, [pts], False, (255,0,0), 2)

        # yeah aight lets try getting the midpoint

        """
        leftmidx = (left_line_x1 + left_line_x2) / 2
        leftmidy = ( (0.65 * img.shape[0]) + img.shape[0]) / 2
        leftmidpoint = int(leftmidx), int(leftmidy)
        cv2.line(img, (left_line_x1, int(0.65 * img.shape[0])), leftmidpoint, leftColor, 10)
        cv2.line(img, (leftmidpoint), (left_line_x2, img.shape[0]), leftColor, 10)
        """

        # cv2.line(img, (right_line_x1, int(0.65 * img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)

        # cv2.line(img, (left_line_x1, int(0.65 * img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
        # cv2.line(img, (right_line_x1, int(0.65 * img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)

        isClosed = True
        color = (255, 0, 0)
        thickness = 10
        print('draaw?')
        # cv2.polylines(img, [pts], isClosed, color, thickness)
        print('draaw?2')



    except ValueError:
        # I keep getting errors for some reason, so I put this here. Idk if the error still persists.
        print("error except valuerror")
        pass

    return [left_line_x1, right_line_x1]


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
    if coors[0] is not None and coors[1] is not None:
        left_diff = int(coors[0])
        right_diff = 635 - int(coors[1])
        padding = 20
        print("left: ", left_diff)
        print("right: ", right_diff)
        # r = 400
        if (right_diff - padding < left_diff) and (left_diff < right_diff + padding):
            print("stay straight")
            direction = "up"
        elif (left_diff < right_diff - padding):
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

    cv2.imshow("img1", interest)  # should return the roi of the frame

    cv2.imshow("img2 (color filter)", filterimg)  # very scary HLS image
    cv2.imshow("img3 (grayscale)", grayimg)  # it is gray now
    cv2.imshow("img4 (canny filter)", canny)  # canny is wack
    cv2.imshow("img5 (hough_lines)", myline)  # myline is wack

    cv2.imshow("main (weighted_img)", weighted_img)

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


# cap = cv2.VideoCapture('project_video.mp4')
# cap = cv2.VideoCapture('harder_challenge_video.mp4') # VIDEO

cap = cv2.VideoCapture(0) # use cv2.VideoCapture(0) to access camera, keep like this for pictures # IMAGE
while (cap.isOpened()):
    _, frame = cap.read()
    frame2 = frame[0:500, 0:600]

    path = r'C:\Users\NinJa\PycharmProjects\testCamera\curvedline2.png' # IMAGE
    image = cv2.imread(path)  # IMAGE
    processImage(image)       # FOR IMAGES

    # processImage(frame)  # FOR VIDEOS

    # final = processImage(frame2) # in case "finaL" is needed
    # processImage(frame2) # video fail (dont use for vid) (keep for camera filming use)     # call processImage

    # CURVED LINES
    # path = r'C:\Users\NinJa\PycharmProjects\testCamera\curvedline1.png' # IMAGE
    # path = r'C:\Users\NinJa\PycharmProjects\testCamera\croppedcurvedline1.png' # IMAGE
    # path = r'C:\Users\NinJa\PycharmProjects\testCamera\goodcurvedline.png' # IMAGE

    # cv2.imshow("hello", image) # IMAGE yeah not relly needed tbh

    # print("hello")
    # cv2.imshow("final", final)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
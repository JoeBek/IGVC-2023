import cv2 
import numpy as np


def canny(img):
    if img is None:
        cap.release() 
        cv2.destroyAllWindows()
        exit()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kernel = 15
    blur = cv2.GaussianBlur(gray, (kernel, kernel), 0)
    canny = cv2.Canny(gray, 50, 150) #should prob be blur
    return canny 


def region_of_interest(canny):
    height = canny.shape[0]
    width = canny.shape[1]
    mask = np.zeros_like(canny)

    #triangle helps focus on the area with lanes
    #may need to modify size based on camera placement
    triangle = np.array([[
        (0, height), 
        (480,130), 
        (1000,height),
    ]], np.int32)

    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

def houghLines(cropped_canny):
    return cv2.HoughLinesP(cropped_canny, 200, np.pi/180, 100, None, minLineLength=40, maxLineGap=25) #ISSUE MIGHT BE HERE



def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    if lines is None:
        return None


    for line in lines:
        for x1, y1, x2, y2 in line:
            fit = np.polyfit((x1,x2), (y1,y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        
    

    left_fit_avg = np.average(left_fit, axis=0)
    right_fit_avg = np.average(right_fit, axis=0)
    left_line = make_points(image, left_fit_avg)
    right_line = make_points(image, right_fit_avg)
    averaged_lines = [left_line, right_line]
    return averaged_lines

def add_weight(frame, line_image):
    return cv2.addWeighted(frame, 0.8, line_image, 1, 1)

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1,y1), (x2,y2), (0, 0, 255), 10)

    return line_image


def make_points(image, line):
    #slope,intercept = line
    try:
        slope, intercept = line
    except TypeError:
        slope, intercept = 0,0
    y1 = int(image.shape[0])
    y2 = int(y1*3.0/5)
    if slope == 0 or slope < (-1e-5):
        x1 = 0
        x2 = 500
  
    else:
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
    '''print(x1)
    print(y1)
    print(x2)
    print(y2)
    print('done')'''

    if x2 > x1:
        temp = x1
        x1 = x2
        x2 = temp
    return [[x1, y1, x2, y2]]

cap = cv2.VideoCapture("IMG_2024.MOV") #use cv2.VideoCapture(0) to access camera
while(cap.isOpened()):
    _, frame = cap.read()
    #cv2.imshow("vid", frame)
    cropped_img = frame[900:1300, :]
    #cropped_img = frame[: 1700:10]
    frame = cropped_img
    canny_image = canny(frame)
    #to show the canny image use: 
    #cv2.imshow("canny_image", canny_image)
    cropped_canny = region_of_interest(canny_image) #cropping image bc lane will be in bottom of image
    #cv2.imshow("cropped_canny", cropped_canny)
    #lines = houghLines(cropped_canny)
    lines = houghLines(canny_image)
    avg_lines = average_slope_intercept(frame, lines)


    line_image = display_lines(frame, avg_lines)
    final_image = add_weight(frame, line_image)

    left_line = avg_lines[0][0]
    right_line = avg_lines[1][0]
    left_x = left_line[0]
    right_x = right_line[0]
    robot_pos = (right_x + left_x) / 2

    robot_dot = cv2.circle(final_image, (int(robot_pos), 300), radius = 5, color=(0, 255, 0), thickness=5)

    center = cv2.circle(final_image, (500, 350), radius = 5, color=(255, 255, 0), thickness=5)

    #if int(robot_pos) > int(right_line[2]):
        #break
    cv2.imshow("result", robot_dot)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


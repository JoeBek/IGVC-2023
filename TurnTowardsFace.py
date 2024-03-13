import cv2
import serial
import time
SerialObj = serial.Serial('COM4') # COMxx  format on Windows
                  # ttyUSBx format on Linux
SerialObj.baudrate = 115200  # set Baud rate to 9600
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1
#time.sleep(1)

face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

video_capture = cv2.VideoCapture()
video_capture.open(1)
print("Start Motor")

initialTime = time.time_ns()
currTime = time.time_ns()
prevTime = time.time_ns()
currentlyDetecting = False

motorCommand = "go"

startOfDetection = time.time_ns()

scalingFactor = 0.2

#CENTER OF SCREEN AT 320 POSSIBLY?

while video_capture.isOpened():
    faceCenterX = None
    offsetFromCenter = None #Positive values are to the right of the camera, negative values are to the left
    faceCenterY = None

    currTime = time.time_ns()
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    image_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = face_detector.detectMultiScale(image_gray, minSize=(50,50))

    # Draw a rectangle around the faces
    largestFace = 0
    for (x, y, w, h) in detections:
        #print(w, h)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        faceSize = w*h
        if (faceSize > largestFace):
            faceCenterX = x+(w/2)
            offsetFromCenter = faceCenterX - 320
            largestFace = faceSize
            #print("Offset")
            #print(offsetFromCenter)


    # Display the resulting frame
    cv2.imshow('Video', frame)

    if len(detections) == 1:
        #print("Detecting Face")
        currentlyDetecting = True
    else: 
        #print("No Detections")
        startOfDetection = time.time_ns()
        currentlyDetecting = False

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if (currentlyDetecting and (currTime - startOfDetection) > 100000000):
        motorCommand = "stop"

    motorCommandString1 = "!G 1 "
    motorCommandString2 = "!G 2 "

    if (offsetFromCenter != None):
        motorSpeedValue1 = int(200 - (scalingFactor * offsetFromCenter))
        motorSpeedValue2 = int(-200 - (scalingFactor * offsetFromCenter))
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
    else:
        motorCommandString1 += "0\r"
        motorCommandString2 += "0\r"  

    if (currTime > prevTime + 100000000): #Send a new command every 0.5 seconds
        prevTime = currTime
        print("Sending Command!")
        #bytes(test_string, 'utf-8')
        SerialObj.write(bytes(motorCommandString1, 'utf-8'))
        SerialObj.write(bytes(motorCommandString2, 'utf-8'))
        print(bytes(motorCommandString1, 'utf-8'))
        print(bytes(motorCommandString2, 'utf-8'))


# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()
SerialObj.close()      # Close the port
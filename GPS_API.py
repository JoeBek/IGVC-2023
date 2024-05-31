# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
import math

def degToRad(angle):
    radianAngle = angle * math.pi/180
    return radianAngle

class GPS:
    def __init__(self, comPort, waypointLatitude, waypointLongitude):
        self.SerialObj = serial.Serial(comPort) # COMxx  format on Windows, ttyUSBx format on Linux
        self.SerialObj.baudrate = 112500  # set Baud rate to 9600
        self.SerialObj.bytesize = 8   # Number of data bits = 8
        self.SerialObj.parity  ='N'   # No parity
        self.SerialObj.stopbits = 1   # Number of Stop bits = 1
        self.SerialObj.timeout = 0.25
        self.destinationLocation = {
            "latitude" : waypointLatitude,
            "longitude" : waypointLongitude
        }
        self.currentLocation = {
            "latitude" : None,
            "longitude" : None
        }
        self.validPosition = False
        self.currentMessage = "Nothing"
        time.sleep(0.25)
        self.SerialObj.write(b"UNLOGALL\r\n")
        time.sleep(0.25)
        self.SerialObj.write(b"log bestpos ontime 2\r\n")

    def updateWaypoint(self, newLatitude, newLongitude):
        self.destinationLocation = {
            "latitude" : newLatitude,
            "longitude" : newLongitude
        }

    def findCompassBearing(self):
        if not(self.currentLocation["latitude"] == None):            
            Yval = math.sin(degToRad(self.destinationLocation["longitude"] - self.currentLocation["longitude"])) * math.cos(degToRad(self.destinationLocation["latitude"]))
            Xval = ( math.cos(degToRad(self.currentLocation["latitude"])) * math.sin(degToRad(self.destinationLocation["latitude"])) ) - ( math.sin(degToRad(self.currentLocation["latitude"])) * math.cos(degToRad(self.destinationLocation["latitude"])) * math.cos(degToRad(self.destinationLocation["longitude"] - self.currentLocation["longitude"])) )
            heading = math.atan2(Yval, Xval)
            heading = ((heading * (180/math.pi)) + 360) % 360 #convert from radians to degrees
            return heading
        else:
            return None

    def findWaypointDistance(self):
        if not(self.currentLocation["latitude"] == None): 
            X = degToRad(self.destinationLocation["longitude"] - self.currentLocation["longitude"]) * math.cos(degToRad((self.currentLocation["latitude"] + self.destinationLocation["latitude"])/2))
            Y = degToRad(self.destinationLocation["latitude"] - self.currentLocation["latitude"])
            dist = math.sqrt(X*X + Y*Y) * 6371000
            return dist
        else:
            return None

    def updatePosition(self):
        if (self.SerialObj.inWaiting() > 0):
            SerialString = self.SerialObj.readline()
            SerialString = str(SerialString)
            SerialStringList = SerialString.split()
            if ("SOL_COMPUTED" in SerialString):
                self.currentLocation["latitude"] = float(SerialStringList[3])
                self.currentLocation["longitude"] = float(SerialStringList[4])
                self.validPosition = True

    def closeGPS(self):
        self.SerialObj.close()
        

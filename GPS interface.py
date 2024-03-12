# Python code transmits a byte to Arduino /Microcontroller
import time
from GPS_API import GPS

gps = GPS("COM7", 40, -80)

while True:

    gps.updatePosition()

    if (gps.validPosition):
        distToWaypoint = gps.findWaypointDistance()
        if (distToWaypoint > 2):
            print("Desired Heading: " + str(gps.findCompassBearing()))
            print("Distance to waypoint: " + str(distToWaypoint))
        else:
            print("AT THE WAYPOINT!!!!!!!!!!!!!!!!!!")

# February 9, 2024
# Authors: Harrison Bui, John Shebey
#
#

import pyzed.sl as sl
import math

# Create a ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.INCH

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Failed to open camera. Error code:", err)
    exit(1)

# move forward
print("Moving forward")

image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()
runtime_params = sl.RuntimeParameters()
depth_value = 100

# initialize past readings
THRESHOLD = 25
NUM_OF_READINGS = 10
mostRecentReadings = [100] * NUM_OF_READINGS

# do while distance < THRESHOLD
avg = THRESHOLD * 100
while avg > THRESHOLD:
    # Grab an image
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS
        zed.retrieve_image(image, sl.VIEW.LEFT)  # Get the left image
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)  # Retrieve depth matrix. Depth is aligned on the left RGB image
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    # Get and print distance value in mm at the center of the image
    x = int(image.get_width() / 2)
    y = int(image.get_height() / 2)
    err, depth_value = depth.get_value(x, y)

    if depth_value == float('-inf'):
        depth_value = 0

    if err == sl.ERROR_CODE.SUCCESS:
        if not math.isnan(depth_value) and depth_value != float('inf') and depth_value != float('-inf'):
            mostRecentReadings.pop(0)
            mostRecentReadings.append(depth_value)
            sum = 0
            for idx in range(NUM_OF_READINGS):
                sum += mostRecentReadings[idx]
            avg = 1.0 * sum / NUM_OF_READINGS
        print("Distance to Camera at ({0}, {1}): {2} in".format(x, y, depth_value))
    else:
        print("Failed to capture depth. Error code:", err)

# stop
print("Stopped")

# cleanup
zed.close()

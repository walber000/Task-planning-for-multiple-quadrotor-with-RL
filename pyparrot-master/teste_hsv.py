import cv2
import os
import numpy as np
import time
#os.chdir(r'E:\Documentos\Mestrado')
from imutils.video import WebcamVideoStream
from imutils.video import FPS

def nothing(x):
    pass

# url1 = "http://161.24.19.89/mjpg/video.mjpg"
# url1 = "http://192.168.1.105:8080/video" #eug
# url1 = "http://192.168.1.114:8080/video"
# url1 = "http://192.168.1.106:8080/video" # me

#url = "http://192.168.0.81/img/video.mjpeg"
# url = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"
url = "http://161.24.19.89/mjpg/video.mjpg"

stream1 = WebcamVideoStream(src=url).start()

# Load in image
#image = cv2.imread('http://161.24.19.89/axis-cgi/jpg/image.cgi')
image = stream1.read()

# Create a window
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('SMin','image',0,255,nothing)
cv2.createTrackbar('VMin','image',0,255,nothing)
cv2.createTrackbar('HMax','image',0,179,nothing)
cv2.createTrackbar('SMax','image',0,255,nothing)
cv2.createTrackbar('VMax','image',0,255,nothing)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

output = image
wait_time = 33

while(1):

	#image = cv2.imread('http://161.24.19.89/axis-cgi/jpg/image.cgi')
	image = stream1.read()
	# output = cv2.resize(image, (240,320))

	# get current positions of all trackbars
	hMin = cv2.getTrackbarPos('HMin','image')
	sMin = cv2.getTrackbarPos('SMin','image')
	vMin = cv2.getTrackbarPos('VMin','image')

	hMax = cv2.getTrackbarPos('HMax','image')
	sMax = cv2.getTrackbarPos('SMax','image')
	vMax = cv2.getTrackbarPos('VMax','image')

	# Set minimum and max HSV values to display
	lower = np.array([hMin, sMin, vMin])
	upper = np.array([hMax, sMax, vMax])

	# Create HSV Image and threshold into a range.
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower, upper)
	output = cv2.bitwise_and(image,image, mask= mask)

    # Print if there is a change in HSV value
	if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
		print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
		phMin = hMin
		psMin = sMin
		pvMin = vMin
		phMax = hMax
		psMax = sMax
		pvMax = vMax

	# Display output image
	cv2.namedWindow('image2', cv2.WINDOW_NORMAL)
	cv2.resizeWindow('image2', 800, 600)
	cv2.imshow("image2",output)

	# Wait longer to prevent freeze for videos.
	if cv2.waitKey(wait_time) & 0xFF == ord('q'):
		break

cv2.destroyAllWindows()
from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import numpy as np
import cv2
from time import sleep
import datetime
from threading import Thread

# url = "http://192.168.0.81/img/video.mjpeg" #teto
url = "http://admin:camadmin@192.168.0.82/img/video.mjpeg" #teto2

# url = "http://192.168.1.105:8080/video"

#url = "http://161.24.19.89/mjpg/video.mjpg" #axis

# camera axis:
# cap = cv2.VideoCapture("http://161.24.19.89/axis-cgi/jpg/image.cgi") #imagem
# url = "http://161.24.19.89/mjpg/video.mjpg"
# cap = cv2.VideoCapture("http://161.24.19.207:8081/img/snapshot.cgi") #imagem

# created a *threaded* video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src=url).start()
#fps = FPS().start()
 
# loop over some frames...this time using the threaded stream
while (1):
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	rows, cols, ch = frame.shape
	print(rows," ",cols," ",ch)
	#frame = imutils.resize(frame, width=400)

	#find color BGR to HSV
	# green = np.uint8([[[0,255,0 ]]])
	# hsv_green = cv.cvtColor(green,cv.COLOR_BGR2HSV)
	# print( hsv_green ) # [H-10, 100,100] and [H+10, 255, 255] 
 
    # Converts images from BGR to HSV 
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
	
	lower_blue = np.array([84,42,145]) #old = 84 60 80
	upper_blue = np.array([132,255,255]) 
	
#	lower_blue = np.array([110,50,50]) 
#	upper_blue = np.array([130,255,255]) 

	lower_ylw = np.array([23,100,140]) 
	upper_ylw = np.array([45,240,250])

	lower_green = np.array([40,50,50]) 
	upper_green = np.array([75,255,255])
	
	lower_red1 = np.array([0,50,50]) 
	upper_red1 = np.array([10,255,255])
	
	lower_red2 = np.array([172,50,50]) 
	upper_red2 = np.array([179,255,255])
	
	mask_green = cv2.inRange(hsv, lower_green, upper_green)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
	mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
  
	res_green = cv2.bitwise_and(frame,frame, mask= mask_green)
	res_blue = cv2.bitwise_and(frame,frame, mask= mask_blue)
	res_red1 = cv2.bitwise_and(frame,frame, mask= mask_red1)
	res_red2 = cv2.bitwise_and(frame,frame, mask= mask_red2)
	res_ylw = cv2.bitwise_and(frame,frame, mask= mask_ylw)
	res_red = res_red1 + res_red2
	res = res_blue
	
	# cv2.namedWindow('Resized Window', cv2.WINDOW_NORMAL)
	# cv2.resizeWindow('Resized Window', 1280, 720)
	#cv2.imshow('Resized Window',frame) 
	# sub = frame - res
	# cv2.imshow('sub',sub)	
	cv2.imshow('Resized Window',res) 
	if(cv2.waitKey(1)!=-1):
		break
	
	# update the FPS counter
	#fps.update()
 
# stop the timer and display FPS information
#fps.stop()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()	

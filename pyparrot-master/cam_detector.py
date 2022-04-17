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

url = "http://192.168.0.81/img/video.mjpeg" # cam1
# url = "http://admin:camadmin@192.168.0.82/img/video.mjpeg" # cam2
#url = "http://161.24.19.89/mjpg/video.mjpg" # axis

stream = WebcamVideoStream(src=url).start()

x_co = 0
y_co = 0
def on_mouse(event,x,y,flag,param):
	global x_co
	global y_co
	if(event==cv2.EVENT_LBUTTONDOWN):
		print(x,y)
		
		#hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
		# im2 = stream.read()
		#px = im[y,x]
		#hsv[(y-10):(y+10),(x-10):(x+10)]=px
		#cv2.circle(im2,(x,y),5,(0,0,255),5)
		#cv2.imshow("camera2", hsv)
		
		
		# px = hsv[x,y]
		# px[0]=px[0]*360/255
		# px[1]=px[1]*100/255
		# px[2]=px[2]*100/255
		#print(px)

# stream = WebcamVideoStream(src=url).start()
# im2 = stream.read()
# cv2.imshow("camera2", im2)

while(1):
	#le imagem
	im = stream.read()
	
	# pts4 = np.float32([[0,16],[640,16],[640,480],[0,480]]) #cam3
	# pts3 = np.float32([[0,0],[640,0],[640,480],[0,480]]) #output desejado
	# M3 = cv2.getPerspectiveTransform(pts4,pts3)
	# im = cv2.warpPerspective(im,M3,(640,480))
	
	# mtx = np.array([[845, 0, 319.6],[0, 891, 157.3], [0, 0, 1]])
	# dist = np.array([[ 0.265530618, -2.45263550, -0.0382963307, -0.000583787615,   6.56047640]])
	# im = cv2.undistort(im, mtx, dist)
	
#c	print(im.shape)
	cv2.setMouseCallback("camera",on_mouse, 0);
	cv2.imshow("camera", im)
	if cv2.waitKey(10) == 27:
		break
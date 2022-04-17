import numpy as np
import cv2 as cv
import glob
import os.path

from imutils.video import WebcamVideoStream
from threading import Thread

url1 = "http://192.168.0.81/img/video.mjpeg"
url2 = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"

stream1 = WebcamVideoStream(src=url1).start()
stream2 = WebcamVideoStream(src=url2).start()

while(1):
	#le imagem
	img1 = stream1.read()
	img2 = stream2.read()

	#img = cv.imread('calib\calib1.jpg')
	#rows,cols,ch = img.shape

	#old_pts1 = np.float32([[34,16],[558,16],[575,479],[23,479]]) #cam1
	pts1 = np.float32([[24,16],[576,16],[574,479],[35,479]]) #cam1
	#old_pts2 = np.float32([[34,16],[596,16],[578,479],[66,479]]) #cam2
	pts2 = np.float32([[34,16],[598,16],[581,479],[66,479]]) #cam2
	pts3 = np.float32([[0,0],[640,0],[640,480],[0,480]])
	
	M1 = cv.getPerspectiveTransform(pts1,pts3)
	M2 = cv.getPerspectiveTransform(pts2,pts3)
	
	dst1 = cv.warpPerspective(img1,M1,(640,480))
	dst2 = cv.warpPerspective(img2,M2,(640,480))

	cv.imshow('dst1',dst1)
	cv.imshow('dst2',dst2)
	
	if(cv.waitKey(30)!=-1):
		break
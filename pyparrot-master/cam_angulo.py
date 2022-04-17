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
import math

url = "http://192.168.0.81/img/video.mjpeg"
#url = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"

stream = WebcamVideoStream(src=url).start()
im_org = stream.read()

while(1):
	#le imagem
	im = stream.read()
	if(im_org.all != im.all):
		im_org = im

		#recorta imagem
		#im = im[100:300,250:450]
		#im = cv2.resize(im,None,fx=3, fy=3, interpolation = cv2.INTER_CUBIC)
		
		#ajusta imagem
		pts1 = np.float32([[34,16],[596,16],[578,479],[66,479]]) #cam2
		pts1 = np.float32([[34,16],[558,16],[575,479],[23,479]]) #cam1
		pts2 = np.float32([[0,0],[640,0],[640,480],[0,480]])
		M = cv2.getPerspectiveTransform(pts1,pts2)
		im = cv2.warpPerspective(im,M,(640,480))
		
		#contorno de cor especifica apenas
		hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
		
		lower_blue = np.array([104,120,50]) 
		upper_blue = np.array([131,250,250]) 
		mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
		res_blue = cv2.bitwise_and(im,im, mask= mask_blue)
		
		lower_green = np.array([40,120,30]) 
		upper_green = np.array([80,240,240])	
		mask_green = cv2.inRange(hsv, lower_green, upper_green)
		res_green = cv2.bitwise_and(im,im, mask= mask_green)
		
		lower_ylw = np.array([23,100,140]) 
		upper_ylw = np.array([45,240,250])	
		mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
		res_ylw = cv2.bitwise_and(im,im, mask= mask_ylw)
		
		# lower_ylw = np.array([27,80,80]) 
		# upper_ylw = np.array([51,255,255])	
		# mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
		# res_ylw = cv2.bitwise_and(im,im, mask= mask_ylw)
		
		lower_red1 = np.array([0,100,50]) 
		upper_red1 = np.array([10,255,255])
		lower_red2 = np.array([172,100,50]) 
		upper_red2 = np.array([179,255,255])
		mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
		mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
		res_red1 = cv2.bitwise_and(im,im, mask= mask_red1)
		res_red2 = cv2.bitwise_and(im,im, mask= mask_red2)
		
		res = res_ylw
		res2 = res_blue
		
		#gray scale
		#imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
		imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		imgray2 = cv2.cvtColor(res2, cv2.COLOR_BGR2GRAY)

		#aplica threshold
		ret, thresh = cv2.threshold(imgray, 40, 255, 0)
		ret2, thresh2 = cv2.threshold(imgray2, 40, 255, 0)

		#traca contornos
		contours, hierarchy = cv2.findContours(thresh, 1, 2)
		contours2, hierarchy2 = cv2.findContours(thresh2, 1, 2)
		
		#desenha contornos em cima da imagem original
		#cv2.drawContours(im, contours2, -1, (0,255,0), 2)

		size_max = 0
		size_max2 = 0
		i_max = 0
		i_max2 = 0
		i = 0
		i2 = 0
		cx = 10
		cy = 10
		cx2 = 10
		cy2 = 10
		
		if(len(contours)>0):
			for i in range(len(contours)):
				cnt = contours[i]
				size = cv2.contourArea(cnt)
				if size > size_max:
					size_max = size
					i_max = i
		
			cnt=contours[i_max]
			M = cv2.moments(cnt)
			if(M['m00']!=0):
				cx = int(M['m10']/M['m00']) #centroide
				cy = int(M['m01']/M['m00'])
				cv2.circle(im,(cx,cy),2,(0,0,255),2)
			cv2.drawContours(im, cnt, -1, (0,255,0), 2)
			
		if(len(contours2)>0):
			for i2 in range(len(contours2)):
				cnt2 = contours2[i2]
				size2 = cv2.contourArea(cnt2)
				if size2 > size_max2:
					size_max2 = size2
					i_max2 = i2

			cnt2=contours2[i_max2]
			M2 = cv2.moments(cnt2)
			if(M2['m00']!=0):
				cx2 = int(M2['m10']/M2['m00']) #centroide
				cy2 = int(M2['m01']/M2['m00'])
				cv2.circle(im,(cx2,cy2),2,(0,0,255),2)
			cv2.drawContours(im, cnt2, -1, (0,255,0), 2)
			
		#linha
		cv2.line(im,(cx,cy),(cx2,cy2),(0,255,0),2)
		ang = math.atan2((cy-cy2),(cx-cx2))
		ang = ang*180/3.14159
		print("angulo: %.1f" % ang)
		
		#mostrar imagem
		cv2.imshow("res",res)
		cv2.imshow("res2",res2)
		cv2.imshow("imagem original",im)
		#cv2.imshow('partes coloridas',res) 
		if(cv2.waitKey(10)!=-1):
			break
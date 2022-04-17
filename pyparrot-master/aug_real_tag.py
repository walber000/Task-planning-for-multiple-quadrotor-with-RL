#!/usr/bin/env python3

from __future__ import print_function

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)	
import cv2


from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import numpy as np

from time import sleep
import time
import datetime
from threading import Thread
import math




mtx = np.array([[845, 0, 319.6],[0, 891, 157.3], [0, 0, 1]])
dist = np.array([[ 0.265530618, -2.45263550, -0.0382963307,
-0.000583787615,   6.56047640]])

pts1 = np.float32([[28,27],[569,16],[580,469],[36,480]]) #cam1
pts2 = np.float32([[51,16],[620,34],[572,480],[75,462]]) #cam2
pts4 = np.float32([[0,16],[640,16],[640,480],[0,480]]) #cam3
pts3 = np.float32([[0,0],[640,0],[640,480],[0,480]]) #output desejado

url1 = "http://192.168.0.81/img/video.mjpeg"
url2 = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"
url3 = "http://161.24.19.89/mjpg/video.mjpg"
try:
	stream1 = WebcamVideoStream(src=url1).start()
	stream2 = WebcamVideoStream(src=url2).start()
	stream3 = WebcamVideoStream(src=url3).start()
	good = 1
except:
	print("deu ruim")
	good = 0

M1 = cv2.getPerspectiveTransform(pts1,pts3)
M2 = cv2.getPerspectiveTransform(pts2,pts3)
M3 = cv2.getPerspectiveTransform(pts4,pts3)

lower_blue = np.array([84,60,80]) 
upper_blue = np.array([132,255,255]) 

lower_green = np.array([48,70,70]) 
upper_green = np.array([75,255,255])

lower_ylw = np.array([23,100,140]) 
upper_ylw = np.array([45,240,250])

lower_red1 = np.array([0,100,50]) 
upper_red1 = np.array([11,255,255])

lower_red2 = np.array([164,100,50]) 
upper_red2 = np.array([180,255,255])

lower_pink1 = np.array([0,35,120]) 
upper_pink1 = np.array([11,140,255])

lower_pink2 = np.array([140,35,120]) 
upper_pink2 = np.array([180,140,255])

# count = 0; erro1 = 0; erro2 = 0
# time1 = time.perf_counter()

while(good==1):
	try:
		#le imagens
		img1 = stream1.read()
		img2 = stream2.read()
		img3 = stream3.read()
		
		#alterações nas imagens
		img1 = cv2.warpPerspective(img1,M1,(640,480))
		img2 = cv2.warpPerspective(img2,M2,(640,480))
		img3 = cv2.warpPerspective(img3,M3,(640,480))
		img3 = cv2.undistort(img3, mtx, dist)
		img3 = img3[0:420,160:460]
		i = 5
		img3 = cv2.bilateralFilter(img3, i, i * 2, i / 2)
		
		# drone 1
		img1, posX_cm, posY_cm, ang = detect_cam1(img1,posZ_cm)
		if(posX_cm == 0):
			img2, posX_cm, posY_cm, ang = detect_cam2(img2,posZ_cm)
		if(posX_cm != 0):
			img3, posZ_cm, cx, cy = detect_cam3(img3,posY_cm)

		#drone 2
		img1, posX2_cm, posY2_cm, ang2 = detect_cam1_second(img1,posZ2_cm)
		if(posX2_cm == 0):
			img2, posX2_cm, posY2_cm, ang2 = detect_cam2_second(img2,posZ2_cm)
		if(posX2_cm != 0):
			img3, posZ2_cm, cx2, cy2 = detect_cam3_second(img3,posY2_cm)
		
		#obj
		img1, posX3_cm, posY3_cm = detect_cam1_obj(img1,posZ3_cm)
		if(posX3_cm == 0):
			img2, posX3_cm, posY3_cm = detect_cam2_obj(img2,posZ3_cm)
		if(posX3_cm != 0):
			img3, posZ3_cm = detect_cam3_obj(img3,posY3_cm)
		
		#oclusao: drones na mesma altura e no mesmo X
		if(posX_cm != 0 and posX2_cm != 0 and posZ_cm == 0): #oclusao do drone1
			# if(abs(posX_cm-posX2_cm)<10): #os drones estao quase no mesmo X
			posX = cx2 ; posZ = cy2
			zlim_min = 34.5 - 50*(posY_cm+178)/178
			zlim_max = 34.5 + 58*(posY_cm+178)/178
			posZ_cm =  zlim_min + ((480-posZ)*(zlim_max-zlim_min)/480)
			print("oclusao drone 1")
				
		#oclusao drone 2
		if(posX_cm != 0 and posX2_cm != 0 and posZ2_cm == 0): #oclusao do drone1
			# if(abs(posX_cm-posX2_cm)<10): #os drones estao quase no mesmo X
			posX = cx ; posZ = cy
			zlim_min = 34.5 - 50*(posY2_cm+178)/178
			zlim_max = 34.5 + 58*(posY2_cm+178)/178
			posZ2_cm =  zlim_min + ((480-posZ)*(zlim_max-zlim_min)/480)
			print("oclusao drone 2")
		
		print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm   ang = %.1f" % ang)
		# print("X2=%.1f" % posX2_cm,"cm   Y2=%.1f" % posY2_cm, "cm   Z2=%.1f" % posZ2_cm, "cm  ang2 = %.1f" % ang2)
		print("X3=%.1f" % posX3_cm,"cm    Y3=%.1f" % posY3_cm, "cm    Z3=%.1f" % posZ3_cm, "cm\n")
		cv2.imshow("imagem 1",img1)
		cv2.imshow("imagem 2",img2)
		cv2.imshow("imagem 3",img3)
		
		# print("total: ",count,"\nerro1: ",erro1,"\nerro2: ",erro2)
		if(cv2.waitKey(10)!=-1):
			# time2 = time.perf_counter()
			# print("frequency: %.2f" % float(count/(time2-time1)))
			break
	except:
		print("abortado: ", sys.exc_info()[0])
		# print("total: ",count,"\nerro1: ",erro1,"\nerro2: ",erro2)
		# time2 = time.perf_counter()
		# print("frequency: %.2f" % float(count/(time2-time1)))
		break
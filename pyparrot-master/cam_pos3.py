from __future__ import print_function

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove()	
import cv2


from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import numpy as np

import time
from time import sleep
import datetime
from threading import Thread
import math
from tkinter import *
from coursework.droneMapGUI import DroneGUI

class GUI(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self):
		global kill_thread, posX_cm, posY_cm
		gui = DroneGUI()
		gui.length = 1.6
		gui.height = 2.8
		gui.scale_val = 2
		# initialize the internal map
		gui.room_map = np.zeros((int(gui.length * 10), int(gui.height * 10)))
		gui.obstacle_ids = np.zeros((int(gui.length * 10), int(gui.height * 10)), dtype='int')
		gui.factor = 20
		gui.draw_room(gui.length, gui.height)
		sleep(1)
		while(kill_thread==0):
			#tk.update_idletasks()
			gui.root.update()
			sleep(0.1)
			gui.position(posX_cm*2,560-posY_cm*2)

posX_cm = 0; posY_cm = 0; kill_thread = 0
# thread = GUI()
# thread.start()

def detect_cam1(img1, h):
	global posX_cm, posY_cm
	size_max = 0; size_max2 = 0
	i_max = 0; i_max2 = 0

	# i=5 #filtro lento
	# img1 = cv2.bilateralFilter(img1, i, i * 2, i / 2)

	img1 = cv2.warpPerspective(img1,M1,(640,480))
	
	#contorno de cor especifica apenas
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
	res_ylw = cv2.bitwise_and(img1,img1, mask= mask_ylw)
	res = res_ylw
	
	#gray scale
	imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

	#aplica threshold
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)

	#traca contornos
	contours, hierarchy = cv2.findContours(thresh, 1, 2)
	
	#detectar maior contorno da cor 1
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
		else:
			return img1,0,0,0
	
		#roi = img1[cy-30:cy+30,cx-30:cx+30]
		hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
		mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
		res_blue = cv2.bitwise_and(img1,img1, mask= mask_blue)
		imgray2 = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
		ret2, thresh2 = cv2.threshold(imgray2, 40, 255, 0)
		contours2, hierarchy2 = cv2.findContours(thresh2, 1, 2)
		
		#detectar maior contorno da cor 2
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
			else:
				return img1,0,0,0
			
		else:
			return img1,0,0,0
	else:
		return img1,0,0,0

	cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	cv2.drawContours(img1, cnt, -1, (0,255,0), 2)
	cv2.drawContours(img1, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img1,(cx,cy),(cx2,cy2),(0,255,0),2)
	ang = math.atan2((cy-cy2),(cx-cx2))
	ang = ang*180/3.14159
	#print("angulo: %.1f" % ang)
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2
	posY = (cy+cy2)/2
	ymin = 38 #30
	ymax = 174 #171.5
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (90-ymin)*h/hmax
	ylim_max = ymax - (ymax-90)*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm")
	
	return img1, posX_cm, posY_cm, ang

def detect_cam1ang(img1, h):
	global posX_cm, posY_cm
	size_max = 0; size_max2 = 0
	i_max = 0; i_max2 = 0

	# i=5 #filtro lento
	# img1 = cv2.bilateralFilter(img1, i, i * 2, i / 2)

	img1 = cv2.warpPerspective(img1,M1,(640,480))
	
	#contorno de cor especifica apenas
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
	res_ylw = cv2.bitwise_and(img1,img1, mask= mask_ylw)
	res = res_ylw
	
	#gray scale
	imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

	#aplica threshold
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)

	#traca contornos
	contours, hierarchy = cv2.findContours(thresh, 1, 2)
	
	#detectar maior contorno da cor 1
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
		else:
			return img1,0,0,0
	else:
		return img1,0,0,0

	img1 = img1[cy-30:cy+30,cx-30:cx+30]
	img1 = cv2.resize(img1,None,fx=3, fy=3, interpolation = cv2.INTER_CUBIC)
	
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
	res_ylw = cv2.bitwise_and(img1,img1, mask= mask_ylw)
	res = res_ylw
	imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)
	contours, hierarchy = cv2.findContours(thresh, 1, 2)

	#detectar maior contorno da cor 1
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
		else:
			return img1,0,0,0
	
		#roi = img1[cy-30:cy+30,cx-30:cx+30]
		hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
		mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
		res_blue = cv2.bitwise_and(img1,img1, mask= mask_blue)
		imgray2 = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
		ret2, thresh2 = cv2.threshold(imgray2, 40, 255, 0)
		contours2, hierarchy2 = cv2.findContours(thresh2, 1, 2)
		
		#detectar maior contorno da cor 2
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
			else:
				return img1,0,0,0
			
		else:
			return img1,0,0,0
	else:
		return img1,0,0,0


	cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	cv2.drawContours(img1, cnt, -1, (0,255,0), 2)
	cv2.drawContours(img1, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img1,(cx,cy),(cx2,cy2),(0,255,0),2)
	ang = math.atan2((cy-cy2),(cx-cx2))
	ang = ang*180/3.14159
	#print("angulo: %.1f" % ang)
	
		
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2
	posY = (cy+cy2)/2
	ymin = 38 #30
	ymax = 174 #171.5
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (90-ymin)*h/hmax
	ylim_max = ymax - (ymax-90)*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm")
	
	return img1, posX_cm, posY_cm, ang

def detect_cam2(img2, h):
	global posX_cm, posY_cm
	size_max = 0; size_max2 = 0
	i_max = 0; i_max2 = 0
	M2 = cv2.getPerspectiveTransform(pts2,pts3)

	img2 = cv2.warpPerspective(img2,M2,(640,480))
	
	#contorno de cor especifica apenas
	hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)	
	mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
	res_ylw = cv2.bitwise_and(img2,img2, mask= mask_ylw)
	res = res_ylw
	
	#gray scale
	imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

	#aplica threshold
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)

	#traca contornos
	contours, hierarchy = cv2.findContours(thresh, 1, 2)
	
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
		else:
			return img2,0,0,0
		
		#roi = img2[cx-30:cx+30,cy-30:cy+30]
		hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
		mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)	
		res_blue = cv2.bitwise_and(img2,img2, mask= mask_blue)
		imgray2 = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
		ret2, thresh2 = cv2.threshold(imgray2, 40, 255, 0)
		contours2, hierarchy2 = cv2.findContours(thresh2, 1, 2)
		
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
			else:
				return img2,0,0,0
		else:
			return img2,0,0,0
	else:
		return img2,0,0,0
	
	cv2.circle(img2,(cx,cy),2,(0,0,255),2)
	cv2.circle(img2,(cx2,cy2),2,(0,0,255),2)
	cv2.drawContours(img2, cnt, -1, (0,255,0), 2)
	cv2.drawContours(img2, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img2,(cx,cy),(cx2,cy2),(0,255,0),2)
	ang = math.atan2((cy-cy2),(cx-cx2))
	ang = ang*180/3.14159
	#print("angulo: %.1f" % ang)
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2 ; posY = (cy+cy2)/2
	ymin = 104 #126.3
	ymax = 279-32 #(279-11)
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (226-ymin)*h/hmax
	ylim_max = ymax - (ymax-226)*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm")
	
	return img2, posX_cm, posY_cm, ang

def detect_cam3(img3, y):
	global sizemin ; global sizemax
	size_max = 0; size_max2 = 0
	i_max = 0; i_max2 = 0
	
	M3 = cv2.getPerspectiveTransform(pts4,pts3)
	img3 = cv2.warpPerspective(img3,M3,(640,480))
	
	img3 = cv2.undistort(img3, mtx, dist)

	img3 = img3[0:420,160:480]

	i = 5
	img3 = cv2.bilateralFilter(img3, i, i * 2, i / 2)

	#contorno de cor especifica apenas
	hsv = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)	
	res_blue = cv2.bitwise_and(img3,img3, mask= mask_blue)
	mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
	res_red1 = cv2.bitwise_and(img3,img3, mask= mask_red1)
	res_red2 = cv2.bitwise_and(img3,img3, mask= mask_red2)
	res = res_red1 + res_red2
	
	#gray scale
	imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

	#aplica threshold
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)

	#traca contornos
	contours, hierarchy = cv2.findContours(thresh, 1, 2)
	
	#detectar maior contorno da cor 1
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
		else:
			return img3,0
	else:
		return img3,0
	
	#segundo maior contorno
	del contours[i_max]
	if(len(contours)>0):
		for i in range(len(contours)):
			cnt2 = contours[i]
			size = cv2.contourArea(cnt2)
			if size > size_max2:
				size_max2 = size
				i_max2 = i
	
		cnt2=contours[i_max2]
		M = cv2.moments(cnt2)
		if(M['m00']!=0):
			cx2 = int(M['m10']/M['m00']) #centroide
			cy2 = int(M['m01']/M['m00'])
	
	# if(size_max2<80 or size_max2>500):
		# size_max2 = 0
	
	# if(size_max>sizemax):
		# sizemax=size_max
	# if(size_max<sizemin):
		# sizemin=size_max
		
	cv2.circle(img3,(cx,cy),2,(0,0,255),2)
	cv2.drawContours(img3, cnt, -1, (0,255,0), 2)
	if(size_max2>0 and M['m00']!=0):
		cv2.circle(img3,(cx2,cy2),2,(0,0,255),2)
		cv2.drawContours(img3, cnt2, -1, (0,255,0), 2)
		cx = (cx+cx2)/2; cy = (cy+cy2)/2
	
	#posicao corrigida devido a altura
	y = y + 178
	posX = cx ; posZ = cy
	zlim_min = 34.5 - 50*y/178
	zlim_max = 34.5 + 58*y/178
	posZ_cm =  zlim_min + ((480-posZ)*(zlim_max-zlim_min)/480)
	
	return img3, posZ_cm

	
#inicializacao de variaveis
sizemin = 330; sizemax = 0
cx = 0; cy = 0
cx2 = 0; cy2 = 0
i = 0; i2 = 0; i3 = 0
#h = 24.5 #cone = 24.2, tampa = 0.3
hmax = 246

mtx = np.array([[845, 0, 319.6],[0, 891, 157.3], [0, 0, 1]])
dist = np.array([[ 0.265530618, -2.45263550, -0.0382963307,
-0.000583787615,   6.56047640]])

pts1 = np.float32([[31,27],[575,16],[583,469],[39,480]]) #cam1
pts2 = np.float32([[51,16],[620,24],[577,480],[83,472]]) #cam2
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

lower_blue = np.array([104,80,40]) 
upper_blue = np.array([131,250,250]) 

lower_ylw = np.array([23,100,140]) 
upper_ylw = np.array([45,240,250])

# lower_red1 = np.array([0,50,50]) 
# upper_red1 = np.array([10,255,255])

# lower_red2 = np.array([172,50,50]) 
# upper_red2 = np.array([179,255,255])

lower_red1 = np.array([0,76,76]) 
upper_red1 = np.array([10,255,255])

lower_red2 = np.array([165,76,76]) 
upper_red2 = np.array([179,255,255])

posZ_cm = 0

sum1=0;sum2=0;min1=999;min2=999;max1=-999;max2=-999;counter=0
old_posX = 0; old_posY = 0
timer = 0

while(good==1):
	#le imagens
	img1 = stream1.read()
	img2 = stream2.read()
	img3 = stream3.read()
	
	#posZ_cm = 53.5
	dif_h = 0
	
	img11, posX_cm, posY_cm, ang = detect_cam1(img1,posZ_cm+dif_h)
	if((posX_cm and posY_cm and ang) == 0):
		img2, posX_cm, posY_cm, ang = detect_cam2(img2,posZ_cm+dif_h)
	else:
		img2 = cv2.warpPerspective(img2,M2,(640,480))
	if((posX_cm or posY_cm or ang) != 0):
		img3, posZ_cm = detect_cam3(img3,posY_cm)
		#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm   ang = %.1f" % ang)
		#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm   ang = %.1f" % ang)
	else:
		img3 = cv2.warpPerspective(img3,M3,(640,480))
		img3 = cv2.undistort(img3, mtx, dist)
	
	# if(old_posX!=0):
		# velX = 0.01*(posX_cm-old_posX)/(time.time()-timer)
		# velY = 0.01*(posY_cm-old_posY)/(time.time()-timer)
		# if(abs(velX) > 0.2 or abs(velY) > 0.2):
			# print("velX: %.3f"%velX, "velY: %.3f"%velY)
	
	# timer = time.time()
	# old_posX = posX_cm; old_posY = posY_cm
	
	img1, posX_cm, posY_cm, ang2 = detect_cam1ang(img1,posZ_cm+dif_h)
	print("%.2f"%ang, " %.2f"%ang2,"\n")
	sum1 = sum1 + ang
	sum2 = sum2 + ang2
	if(ang<min1):
		min1=ang
	if(ang2<min2):
		min2=ang2
	if(ang>max1):
		max1=ang
	if(ang2>max2):
		max2=ang2
	
	counter=counter+1
	
	cv2.imshow("imagem 1",img1)
	cv2.imshow("imagem 2",img2)
	cv2.imshow("imagem 3",img3)
	if(cv2.waitKey(20)!=-1):
		#print("sizemin: ",sizemin," sizemax: ",sizemax)
		kill_thread = 1
		print("media1: %.2f"%(sum1/counter))
		print("media2: %.2f"%(sum2/counter))
		print("min1: %.2f"%min1, "max1: %.2f"%max1)
		print("min2: %.2f"%min2, "max2: %.2f"%max2)
		break
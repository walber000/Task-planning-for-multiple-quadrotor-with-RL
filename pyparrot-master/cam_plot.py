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

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
posX2_cm = 0; posY2_cm = 0; posZ2_cm = 0; ang2 = 0
posX3_cm = 0; posY3_cm = 0; posZ3_cm = 0

def detect_cam1(img1, h):
	global posX_cm, posY_cm, ang
	size_max = 0; i_max = 0
	
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	res_blue = cv2.bitwise_and(img1,img1, mask= mask_blue)
	imgray = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
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
		if(M['m00']!=0 and size_max > 80):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img1,0,0,0
		
		roi = img1[max((cy-30),0):min((cy+30),480),max((cx-30),0):min((cx+30),640)]
		hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
		res_ylw = cv2.bitwise_and(roi,roi, mask= mask_ylw)
		imgray = cv2.cvtColor(res_ylw, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(imgray, 40, 255, 0)
		contours, hierarchy = cv2.findContours(thresh, 1, 2)
		
		#detectar maior contorno da cor 2
		size_max = 0
		i_max = 0
		if(len(contours)>0):
			for i in range(len(contours)):
				cnt2 = contours[i]
				size = cv2.contourArea(cnt2)
				if size > size_max:
					size_max = size
					i_max = i

			cnt2=contours[i_max]
			M = cv2.moments(cnt2)
			if(M['m00']!=0 and size_max > 80):
				cx2 = int(M['m10']/M['m00']) #centroide
				cy2 = int(M['m01']/M['m00'])
				cx2 = cx2 + max((cx-30),0) #ajustes devido à ROI
				cy2 = cy2 + max((cy-30),0)
			else:
				return img1,0,0,0
			
		else:
			return img1,0,0,0
	else:
		return img1,0,0,0
	
	cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	# cv2.drawContours(img1, cnt, -1, (0,255,0), 2)
	# cv2.drawContours(img1, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img1,(cx,cy),(cx2,cy2),(0,0,255),1)
	ang = math.atan2(-(cy-cy2),-(cx-cx2)) #negativo pois alterou-se a ordem das cores, agr eh azul dps amarelo
	ang = ang*180/3.14159
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2
	posY = (cy+cy2)/2
	ymin = 33 #30
	ymax = 176 #171.5
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (90-ymin)*h/hmax
	ylim_max = ymax - (ymax-90)*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	
	return img1, posX_cm, posY_cm, ang

def detect_cam1_obj(img1, h):
	global posX3_cm, posY3_cm, posZ3_cm
	size_max = 0; i_max = 0

	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)

	mask_pink1 = cv2.inRange(hsv, lower_pink1, upper_pink1)
	mask_pink2 = cv2.inRange(hsv, lower_pink2, upper_pink2)
	res_pink1 = cv2.bitwise_and(img1,img1, mask= mask_pink1)
	res_pink2 = cv2.bitwise_and(img1,img1, mask= mask_pink2)
	res = res_pink1 + res_pink2

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
		if(M['m00']!=0 and size_max > 5):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img1,0,0
		
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	
	#posicao corrigida devido a altura
	posX = cx
	posY = cy
	ymin = 38 #30
	ymax = 174 #171.5
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (90-ymin)*h/hmax
	ylim_max = ymax - (ymax-90)*h/hmax
	posX3_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY3_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	
	return img1, posX3_cm, posY3_cm

def detect_cam2(img2, h):
	global posX_cm, posY_cm, ang
	size_max = 0; i_max = 0
	
	hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)	
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	res_blue = cv2.bitwise_and(img2,img2, mask= mask_blue)
	imgray = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)
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
		if(M['m00']!=0 and size_max > 80):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])		
		else:
			return img2,0,0,0
		
		roi = img2[max((cy-30),0):min((cy+30),480),max((cx-30),0):min((cx+30),640)]
		hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)	
		res_ylw = cv2.bitwise_and(roi,roi, mask= mask_ylw)
		imgray = cv2.cvtColor(res_ylw, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(imgray, 40, 255, 0)
		contours, hierarchy = cv2.findContours(thresh, 1, 2)
		
		size_max = 0
		i_max = 0
		if(len(contours)>0):
			for i in range(len(contours)):
				cnt2 = contours[i]
				size = cv2.contourArea(cnt2)
				if size > size_max:
					size_max = size
					i_max = i
			
			cnt2=contours[i_max]
			M = cv2.moments(cnt2)
			if(M['m00']!=0 and size_max > 80):
				cx2 = int(M['m10']/M['m00']) #centroide
				cy2 = int(M['m01']/M['m00'])
				cx2 = cx2 + max((cx-30),0) #ajustes devido à ROI
				cy2 = cy2 + max((cy-30),0)
			else:
				return img2,0,0,0
		else:
			return img2,0,0,0
	else:
		return img2,0,0,0
	
	cv2.circle(img2,(cx,cy),2,(0,0,255),2)
	cv2.circle(img2,(cx2,cy2),2,(0,0,255),2)
	# cv2.drawContours(img2, cnt, -1, (0,255,0), 2)
	# cv2.drawContours(img2, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img2,(cx,cy),(cx2,cy2),(0,0,255),1)
	ang = math.atan2(-(cy-cy2),-(cx-cx2))
	ang = ang*180/3.14159
	
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
	
	return img2, posX_cm, posY_cm, ang

def detect_cam2_obj(img2, h):
	global posX3_cm, posY3_cm, posZ3_cm
	size_max = 0; i_max = 0

	hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

	mask_pink1 = cv2.inRange(hsv, lower_pink1, upper_pink1)
	mask_pink2 = cv2.inRange(hsv, lower_pink2, upper_pink2)
	res_pink1 = cv2.bitwise_and(img2,img2, mask= mask_pink1)
	res_pink2 = cv2.bitwise_and(img2,img2, mask= mask_pink2)
	res = res_pink1 + res_pink2

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
		if(M['m00']!=0 and size_max > 5):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img2,0,0
		
	cv2.circle(img2,(cx,cy),2,(0,0,255),2)
	
	#posicao corrigida devido a altura
	posX = cx; posY = cy
	ymin = 104 #126.3
	ymax = 279-32 #(279-11)
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (226-ymin)*h/hmax
	ylim_max = ymax - (ymax-226)*h/hmax
	posX3_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY3_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	
	return img2, posX3_cm, posY3_cm

def detect_cam1_second(img1, h):
	global posX2_cm, posY2_cm, ang2
	size_max = 0; i_max = 0
	
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask_green = cv2.inRange(hsv, lower_green, upper_green)
	res_green = cv2.bitwise_and(img1,img1, mask= mask_green)
	imgray = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
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
		if(M['m00']!=0 and size_max > 80):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img1,0,0,0
		
		roi = img1[max((cy-30),0):min((cy+30),480),max((cx-30),0):min((cx+30),640)]
		hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
		res_ylw = cv2.bitwise_and(roi,roi, mask= mask_ylw)
		imgray = cv2.cvtColor(res_ylw, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(imgray, 40, 255, 0)
		contours, hierarchy = cv2.findContours(thresh, 1, 2)
		
		#detectar maior contorno da cor 2
		size_max = 0
		i_max = 0
		if(len(contours)>0):
			for i in range(len(contours)):
				cnt2 = contours[i]
				size = cv2.contourArea(cnt2)
				if size > size_max:
					size_max = size
					i_max = i

			cnt2=contours[i_max]
			M = cv2.moments(cnt2)
			if(M['m00']!=0 and size_max > 80):
				cx2 = int(M['m10']/M['m00']) #centroide
				cy2 = int(M['m01']/M['m00'])
				cx2 = cx2 + max((cx-30),0) #ajustes devido à ROI
				cy2 = cy2 + max((cy-30),0)
			else:
				return img1,0,0,0
			
		else:
			return img1,0,0,0
	else:
		return img1,0,0,0
	
	cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	# cv2.drawContours(img1, cnt, -1, (0,255,0), 2)
	# cv2.drawContours(img1, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img1,(cx,cy),(cx2,cy2),(0,255,0),2)
	ang2 = math.atan2(-(cy-cy2),-(cx-cx2)) #negativo pois alterou-se a ordem das cores, agr eh azul dps amarelo
	ang2 = ang2*180/3.14159
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2
	posY = (cy+cy2)/2
	ymin = 38 #30
	ymax = 174 #171.5
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (90-ymin)*h/hmax
	ylim_max = ymax - (ymax-90)*h/hmax
	posX2_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY2_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	
	return img1, posX2_cm, posY2_cm, ang2

def detect_cam2_second(img2, h):
	global posX2_cm, posY2_cm, ang2
	size_max = 0; i_max = 0
	
	hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)	
	mask_green = cv2.inRange(hsv, lower_green, upper_green)
	res_green = cv2.bitwise_and(img2,img2, mask= mask_green)
	imgray = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)
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
		if(M['m00']!=0 and size_max > 80):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])		
		else:
			return img2,0,0,0
		
		roi = img2[max((cy-30),0):min((cy+30),480),max((cx-30),0):min((cx+30),640)]
		hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)	
		res_ylw = cv2.bitwise_and(roi,roi, mask= mask_ylw)
		imgray = cv2.cvtColor(res_ylw, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(imgray, 40, 255, 0)
		contours, hierarchy = cv2.findContours(thresh, 1, 2)
		
		size_max = 0
		i_max = 0
		if(len(contours)>0):
			for i in range(len(contours)):
				cnt2 = contours[i]
				size = cv2.contourArea(cnt2)
				if size > size_max:
					size_max = size
					i_max = i

			cnt2=contours[i_max]
			M = cv2.moments(cnt2)
			if(M['m00']!=0 and size_max > 80):
				cx2 = int(M['m10']/M['m00']) #centroide
				cy2 = int(M['m01']/M['m00'])
				cx2 = cx2 + max((cx-30),0) #ajustes devido à ROI
				cy2 = cy2 + max((cy-30),0)
			else:
				return img2,0,0,0
		else:
			return img2,0,0,0
	else:
		return img2,0,0,0
	
	cv2.circle(img2,(cx,cy),2,(0,0,255),2)
	cv2.circle(img2,(cx2,cy2),2,(0,0,255),2)
	# cv2.drawContours(img2, cnt, -1, (0,255,0), 2)
	# cv2.drawContours(img2, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img2,(cx,cy),(cx2,cy2),(0,255,0),2)
	ang2 = math.atan2(-(cy-cy2),-(cx-cx2))
	ang2 = ang2*180/3.14159
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2 ; posY = (cy+cy2)/2
	ymin = 104 #126.3
	ymax = 279-32 #(279-11)
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (226-ymin)*h/hmax
	ylim_max = ymax - (ymax-226)*h/hmax
	posX2_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY2_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	
	return img2, posX2_cm, posY2_cm, ang2

def detect_cam3(img3, y):
	size_max = 0; i_max = 0; cx2 = 0; cy2 = 0

	hsv = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)
	mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
	res_red1 = cv2.bitwise_and(img3,img3, mask= mask_red1)
	res_red2 = cv2.bitwise_and(img3,img3, mask= mask_red2)
	res = res_red1 + res_red2
	
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
		if(M['m00']!=0 and size_max > 15):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img3,0,0,0
	else:
		return img3,0,0,0
	
	#segundo maior contorno
	size_max = 0
	del contours[i_max]
	i_max = 0
	if(len(contours)>0):
		for i in range(len(contours)):
			cnt2 = contours[i]
			size = cv2.contourArea(cnt2)
			if size > size_max:
				size_max = size
				i_max = i
	
		cnt2=contours[i_max]
		M = cv2.moments(cnt2)
		if(M['m00']!=0 and size_max > 15):
			cx2 = int(M['m10']/M['m00']) #centroide
			cy2 = int(M['m01']/M['m00'])
		
	cv2.circle(img3,(cx,cy),2,(0,0,255),2)
	cv2.drawContours(img3, cnt, -1, (0,255,0), 2)
	if(cx2 != 0 and cy2 != 0):
		cv2.circle(img3,(cx2,cy2),2,(0,0,255),2)
		cv2.drawContours(img3, cnt2, -1, (0,255,0), 2)
		cx = (cx+cx2)/2; cy = (cy+cy2)/2
	
	#posicao corrigida devido a altura
	y = y + 178
	posX = cx ; posZ = cy
	zlim_min = 34.5 - 48*y/178
	zlim_max = 34.5 + 56*y/178
	posZ_cm =  zlim_min + ((480-posZ)*(zlim_max-zlim_min)/480)
	
	return img3, posZ_cm, cx, cy

def detect_cam3_second(img3, y):
	size_max = 0; i_max = 0; cx2 = 0; cy2 = 0

	hsv = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)	
	res_blue = cv2.bitwise_and(img3,img3, mask= mask_blue)
	res = res_blue
	
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
		if(M['m00']!=0 and size_max > 25):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img3,0,0,0
	else:
		return img3,0,0,0
	
	#segundo maior contorno
	size_max = 0
	del contours[i_max]
	i_max = 0
	if(len(contours)>0):
		for i in range(len(contours)):
			cnt2 = contours[i]
			size = cv2.contourArea(cnt2)
			if size > size_max:
				size_max = size
				i_max = i
	
		cnt2=contours[i_max]
		M = cv2.moments(cnt2)
		if(M['m00']!=0 and size_max > 25):
			cx2 = int(M['m10']/M['m00']) #centroide
			cy2 = int(M['m01']/M['m00'])
		
	cv2.circle(img3,(cx,cy),2,(0,0,255),2)
	cv2.drawContours(img3, cnt, -1, (0,255,0), 2)
	if(cx2 != 0 and cy2 != 0):
		cv2.circle(img3,(cx2,cy2),2,(0,0,255),2)
		cv2.drawContours(img3, cnt2, -1, (0,255,0), 2)
		cx = (cx+cx2)/2; cy = (cy+cy2)/2
	
	#posicao corrigida devido a altura
	y = y + 178
	posX = cx ; posZ = cy
	zlim_min = 34.5 - 50*y/178
	zlim_max = 34.5 + 58*y/178
	posZ2_cm =  zlim_min + ((480-posZ)*(zlim_max-zlim_min)/480)
	
	return img3, posZ2_cm, cx, cy

def detect_cam3_obj(img3, y):
	global posX3_cm, posY3_cm, posZ3_cm
	size_max = 0; i_max = 0; cx = 0; cy = 0

	hsv = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)
	
	mask_pink1 = cv2.inRange(hsv, lower_pink1, upper_pink1)
	mask_pink2 = cv2.inRange(hsv, lower_pink2, upper_pink2)
	res_pink1 = cv2.bitwise_and(img3,img3, mask= mask_pink1)
	res_pink2 = cv2.bitwise_and(img3,img3, mask= mask_pink2)
	res = res_pink1 + res_pink2
	
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
		if(M['m00']!=0 and size_max > 15):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img3,0
	else:
		return img3,0
		
	cv2.circle(img3,(cx,cy),2,(0,0,255),2)
	cv2.drawContours(img3, cnt, -1, (0,255,0), 2)
	
	#posicao corrigida devido a altura
	y = y + 178
	posX = cx ; posZ = cy
	zlim_min = 34.5 - 50*y/178
	zlim_max = 34.5 + 58*y/178
	posZ3_cm =  zlim_min + ((480-posZ)*(zlim_max-zlim_min)/480)
	
	return img3, posZ3_cm


#inicializacao de variaveis
# cx = 0; cy = 0
# cx2 = 0; cy2 = 0
# i = 0
hmax = 246

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

# plt.style.use('ggplot')

# # https://makersportal.com/blog/2018/8/14/real-time-graphing-in-python
# def live_plotter(x_vec,y1_data,line1,identifier='',pause_time=0.01):
    # if line1==[]:
        # # this is the call to matplotlib that allows dynamic plotting
        # plt.ion()
        # fig = plt.figure(figsize=(13,6))
        # ax = fig.add_subplot(311)
        # # create a variable for the line so we can later update it
        # line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)        
        # #update plot label/title
        # plt.ylabel('Y Label')
        # plt.title('Title: {}'.format(identifier))
        # plt.show()
    
    # # after the figure, axis, and line are created, we only need to update the y-data
    # line1.set_ydata(y1_data)
    # # adjust limits if new data goes beyond bounds
    # if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        # plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
    # # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    # plt.pause(pause_time)
    
    # # return line so we can update it again in the next iteration
    # return line1

# size = 100
# x_vec = np.linspace(0,1,size+1)[0:-1]
# y_vec = np.random.randn(len(x_vec))
# line1 = []; line2 = []; line3 = []

counter = []; X = []; Y = []; Z = []
fig = plt.figure(figsize=(9, 3))
ax1 = fig.add_subplot(1,3,1)
ax2 = fig.add_subplot(1,3,2)
ax3 = fig.add_subplot(1,3,3)
count = 1

def animate(i):
	global count, posX_cm, posY_cm, posZ_cm
	counter.append(count)
	X.append(posX_cm)
	Y.append(posY_cm)
	Z.append(posZ_cm)
	ax1.clear()
	ax1.plot(counter, X)
	ax2.clear()
	ax2.plot(counter, Y)
	ax3.clear()
	ax3.plot(counter, Z)
	count = count + 1

class plot(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self):
		ani = animation.FuncAnimation(fig, animate, interval=1)
		

thread = plot()
thread.start()

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
		# print("X3=%.1f" % posX3_cm,"cm    Y3=%.1f" % posY3_cm, "cm    Z3=%.1f" % posZ3_cm, "cm\n")
		# cv2.imshow("imagem 1",img1)
		# cv2.imshow("imagem 2",img2)
		# cv2.imshow("imagem 3",img3)

		# y_vec[-1] = posX_cm
		# line1 = live_plotter(x_vec,y_vec,line1)
		# y_vec = np.append(y_vec[1:],0.0)

		# counter.append(count)
		# X.append(float(posX_cm))
		# Y.append(float(posY_cm))
		# Z.append(float(posZ_cm))
		
		# ax1.clear()
		# ax1.plot(counter, X)
		# ax2.clear()
		# ax2.plot(counter, Y)
		# ax3.clear()
		# ax3.plot(counter, Z)
		# plt.show()
		# plt.pause(0.00001)
		

		# count = count + 1






		
		# print("total: ",count,"\nerro1: ",erro1,"\nerro2: ",erro2)
		# if(cv2.waitKey(10)!=-1):
			# time2 = time.perf_counter()
			# print("frequency: %.2f" % float(count/(time2-time1)))
			# break
	except:
		print("abortado: ", sys.exc_info()[0])
		# print("total: ",count,"\nerro1: ",erro1,"\nerro2: ",erro2)
		# time2 = time.perf_counter()
		# print("frequency: %.2f" % float(count/(time2-time1)))
		break
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

def detect_cam1(img1, h):
	size_max = 0; size_max2 = 0
	i_max = 0; i_max2 = 0

	img1 = cv2.warpPerspective(img1,M1,(640,480))
	
	#contorno de cor especifica apenas
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)	
	mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
	res_blue = cv2.bitwise_and(img1,img1, mask= mask_blue)
	res_ylw = cv2.bitwise_and(img1,img1, mask= mask_ylw)
	res = res_ylw
	res2 = res_blue
	
	#gray scale
	imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
	imgray2 = cv2.cvtColor(res2, cv2.COLOR_BGR2GRAY)

	#aplica threshold
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)
	ret2, thresh2 = cv2.threshold(imgray2, 40, 255, 0)

	#traca contornos
	contours, hierarchy = cv2.findContours(thresh, 1, 2)
	contours2, hierarchy2 = cv2.findContours(thresh2, 1, 2)
	
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
	
	#cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	#cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	#cv2.drawContours(img1, cnt, -1, (0,255,0), 2)
	#cv2.drawContours(img1, cnt2, -1, (0,255,0), 2)
	
	#linha / angulo
	cv2.line(img1,(cx,cy),(cx2,cy2),(0,255,0),2)
	ang = math.atan2((cy-cy2),(cx-cx2))
	ang = ang*180/3.14159
	#print("angulo: %.1f" % ang)
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2 ; posY = (cy+cy2)/2
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = 20 + 70*h/hmax
	ylim_max = 158 - 68*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640) #640 pixels = 164 cm
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480) #480 pixels = 20-158 cm
	#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm")
	
	return img1, posX_cm, posY_cm, ang
	#return img1, posX, posY, ang

def detect_cam2(img2, h):
	size_max = 0; size_max2 = 0
	i_max = 0; i_max2 = 0
	M2 = cv2.getPerspectiveTransform(pts2,pts3)

	img2 = cv2.warpPerspective(img2,M2,(640,480))
	
	#contorno de cor especifica apenas
	hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)	
	mask_ylw = cv2.inRange(hsv, lower_ylw, upper_ylw)
	res_blue = cv2.bitwise_and(img2,img2, mask= mask_blue)
	res_ylw = cv2.bitwise_and(img2,img2, mask= mask_ylw)
	res = res_ylw
	res2 = res_blue
	
	#gray scale
	imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
	imgray2 = cv2.cvtColor(res2, cv2.COLOR_BGR2GRAY)

	#aplica threshold
	ret, thresh = cv2.threshold(imgray, 40, 255, 0)
	ret2, thresh2 = cv2.threshold(imgray2, 40, 255, 0)

	#traca contornos
	contours, hierarchy = cv2.findContours(thresh, 1, 2)
	contours2, hierarchy2 = cv2.findContours(thresh2, 1, 2)
	
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
	
	#cv2.circle(img2,(cx,cy),2,(0,0,255),2)
	#cv2.circle(img2,(cx2,cy2),2,(0,0,255),2)
	#cv2.drawContours(img2, cnt, -1, (0,255,0), 2)
	#cv2.drawContours(img2, cnt2, -1, (0,255,0), 2)
	
	#linha / angulo
	cv2.line(img2,(cx,cy),(cx2,cy2),(0,255,0),2)
	ang = math.atan2((cy-cy2),(cx-cx2))
	ang = ang*180/3.14159
	#print("angulo: %.1f" % ang)
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2 ; posY = (cy+cy2)/2
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = 124 + 102*h/hmax
	ylim_max = 266 - 40*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm")
	
	return img2, posX_cm, posY_cm, ang

#inicializacao de variaveis
cx = 0; cy = 0
cx2 = 0; cy2 = 0
i = 0; i2 = 0
#h = 24.5 #cone = 24.2, tampa = 0.3
hmax = 246

#ajuste para incluir "bordas" fora do tablado
bXmin1 = 0#24
bXmax1 = 0#64
bYmin1 = 4
bXmin2 = 0#34
bXmax2 = 0#42
bYmin2 = 9
pts1 = np.float32([[24-bXmin1,16+bYmin1],[576+bXmax1,16],[574+bXmax1,479],[35-bXmin1,479]]) #cam1
pts2 = np.float32([[34-bXmin2,16+bYmin2],[598+bXmax2,16],[581+bXmax2,479],[66-bXmin2,479]]) #cam2
pts3 = np.float32([[0,0],[640,0],[640,480],[0,480]]) #output desejado

url1 = "http://192.168.0.81/img/video.mjpeg"
url2 = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"

stream1 = WebcamVideoStream(src=url1).start()
stream2 = WebcamVideoStream(src=url2).start()

M1 = cv2.getPerspectiveTransform(pts1,pts3)
M2 = cv2.getPerspectiveTransform(pts2,pts3)

lower_blue = np.array([104,120,50]) 
upper_blue = np.array([131,250,250]) 

lower_ylw = np.array([23,100,140]) 
upper_ylw = np.array([45,240,250])
	
while(1):
	#le imagens
	# e1 = cv2.getTickCount()
	
	img1 = stream1.read()
	img2 = stream2.read()

	h = 48.7
	
	img1, posX_cm, posY_cm, ang = detect_cam1(img1,h)
	#img1, posX, posY, ang = detect_cam1(img1,h=0)
	if((posX_cm and posY_cm and ang) == 0):
	#if((posX and posY and ang) == 0):
		img2, posX_cm, posY_cm, ang = detect_cam2(img2,h)
	else:
		img2 = cv2.warpPerspective(img2,M2,(640,480))
	if((posX_cm or posY_cm or ang) != 0):
		print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm   ang = %.1f" % ang)
	
	#recorta imagem
	# cutXmin = max(0,round(posX)-50)
	# cutXmax = min(640, round(posX)+50)
	# cutYmin = max(0,round(posY)-50)
	# cutYmax = min(480, round(posY)+50)
	# img3 = img1[cutYmin:cutYmax,cutXmin:cutXmax]
	# img3 = cv2.resize(img3,None,fx=3, fy=3, interpolation = cv2.INTER_CUBIC)
	
	# img3, posX_cm, posY_cm, ang2 = detect_cam1(img3,h=0)
	# print("ang1: %.1f" %ang, "  ang2: %.1f" %ang2)
	
	# x=120
	# img3 = np.zeros((960,640,3), np.uint8)
	# img3[0:480, 0:640] = img2
	# img3[480:960-x, 0:640] = img1[x:480, 0:640]
	# img3 = imutils.rotate_bound(img3, 90)
	
	# rows,cols, ch = img3.shape
	# M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
	# img3 = cv2.warpAffine(img3,M,(cols,rows))
	# img3 = img3[0:640,0:960]
	
	#cv2.imshow("res",res)
	#cv2.imshow("res2",res2)
	cv2.imshow("imagem 1",img1)
	cv2.imshow("imagem 2",img2)
	#cv2.imshow("imagem 3",img3)
	if(cv2.waitKey(20)!=-1):
		break
		
	# e2 = cv2.getTickCount()
	# print((e2 - e1)/cv2.getTickFrequency())

cv2.destroyAllWindows()	
stream1.stop()
stream2.stop()
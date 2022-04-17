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

url = "http://192.168.0.81/img/video.mjpeg"
stream = WebcamVideoStream(src=url).start()
fps = FPS().start()
im_org = stream.read()

while(1):
	#le imagem
	
	im = stream.read()
	
	if(im_org.all != im.all):
		im_org = im
		#im = im[0:200,300:400]
		#im = cv2.resize(im,None,fx=3, fy=3, interpolation = cv2.INTER_CUBIC)

		#recorta imagem
		#im = im[0:480,0:300]

		#contorno de cor especifica apenas
		hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV) 

		lower_red1 = np.array([0,50,50]) 
		upper_red1 = np.array([10,255,255])
		lower_red2 = np.array([172,100,100]) 
		upper_red2 = np.array([179,255,255])
		mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
		mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
		res_red1 = cv2.bitwise_and(im,im, mask= mask_red1)
		res_red2 = cv2.bitwise_and(im,im, mask= mask_red2)

		lower_blue = np.array([105,60,60]) 
		upper_blue = np.array([130,240,200]) 
		mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
		res_blue = cv2.bitwise_and(im,im, mask= mask_blue)

		res_red = res_red1 + res_red2
		res = res_red

		#gray scale
		#imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
		imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

		#aplica threshold
		ret, thresh = cv2.threshold(imgray, 40, 240, 0)

		#traca contornos
		contours, hierarchy = cv2.findContours(thresh, 1, 2)

		#desenha contornos em cima da imagem original
		#cv2.drawContours(im, contours, -1, (0,255,0), 2)

		size_max = 0
		size_max2 = 0
		i_max = 0
		i_max2 = 0
		i=0

		if(len(contours)>0):
			for i in range(len(contours)):
				cnt = contours[i]
				size = cv2.arcLength(cnt,True)
				if size > size_max:
					size_max = size
					i_max = i

			# for i in range(len(contours)):
				# cnt = contours[i]
				# size = cv2.arcLength(cnt,True)
				# if (size > size_max2)and(size!=size_max):
					# size_max2 = size
					# i_max2 = i

			cnt=contours[i_max]
			M = cv2.moments(cnt)
			if(M['m00']!=0):
				cx = int(M['m10']/M['m00']) #centroide
				cy = int(M['m01']/M['m00'])
				cv2.circle(im,(cx,cy),3,(0,0,255),3)

		if(size_max>100):
			cv2.drawContours(im, cnt, -1, (0,255,0), 2)
			print("size: %.2f" % size_max)
		#retangulo
		#x,y,w,h = cv2.boundingRect(cnt)
		#cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)

		#menor retangulo
		#rect = cv2.minAreaRect(cnt)
		#box = cv2.boxPoints(rect)
		#box = np.int0(box)
		#cv2.drawContours(im,[box],0,(0,0,255),2)

		#mostrar imagem
		cv2.imshow("res",res)
		cv2.imshow("thresh",thresh)
		cv2.imshow("imagem original",im)
		#cv2.imshow('partes coloridas',res) 

		if(cv2.waitKey(30)!=-1):
			break

		fps.update()
		fps.stop()
		print("FPS: {:.2f}".format(fps.fps()))
		print("")

fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# # do a bit of cleanup
# cv2.destroyAllWindows()
# vs.stop()	

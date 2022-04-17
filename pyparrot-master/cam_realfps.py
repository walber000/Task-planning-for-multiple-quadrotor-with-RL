#!/usr/bin/env python3

from __future__ import print_function

import sys	
import cv2
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import imutils
import numpy as np

from time import sleep
import time
from threading import Thread
import math

url1 = "http://192.168.0.81/img/video.mjpeg"
url2 = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"
url3 = "http://161.24.19.89/mjpg/video.mjpg"

try:
	stream1 = WebcamVideoStream(src=url1).start()
	# stream2 = WebcamVideoStream(src=url2).start()
	# stream3 = WebcamVideoStream(src=url3).start()
	good = 1
except:
	print("deu ruim")
	good = 0

count = 0
time1 = time.perf_counter()
img1_old = stream1.read()
# img2_old = stream2.read()
# img3_old = stream3.read()
sleep(1)

while(good==1):
	try:
		#le imagens
		img1 = stream1.read()
		# img2 = stream2.read()
		# img3 = stream3.read()
		
		if(img1.all!=img1_old.all):
			count = count + 1
			img1_old = img1
			
		print(count)
		# if(cv2.waitKey(1)!=-1):
			# print("total: ",count,"\nerro1: ",erro1,"\nerro2: ",erro2)
			# time2 = time.perf_counter()
			# print("frequency: %.2f" % float(count/(time2-time1)))
			# break
	except:
		#print("total: ",count,"\nerro1: ",erro1,"\nerro2: ",erro2)
		time2 = time.perf_counter()
		print("frequency: %.2f" % float(count/(time2-time1)))
		break
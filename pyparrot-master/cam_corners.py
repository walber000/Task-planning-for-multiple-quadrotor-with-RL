import numpy as np
import cv2 as cv
from imutils.video import WebcamVideoStream

url1 = "http://192.168.0.81/img/video.mjpeg"
url2 = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"
stream1 = WebcamVideoStream(src=url1).start()
stream2 = WebcamVideoStream(src=url2).start()

def detect1(edges):
	i = 0; pos1 = 0; pos2 = 0; pos3 = 0; pos4 = 0;
	while(pos1 == 0):
		pos1 = edges[0,i]; i=i+1
	pos1 = i
	i = 0
	while(pos2 == 0):
		pos2 = edges[0,639-i]; i=i+1
	pos2 = 639-i
	i = 0
	while(pos3 == 0):
		pos3 = edges[463,i]; i=i+1
	pos3 = i
	i = 0
	while(pos4 == 0):
		pos4 = edges[463,639-i]; i=i+1
	pos4 = 639-i
		
	print("cam1:")
	print("   pos1:",pos1," pos2:",pos2," pos3:",pos3," pos4:",pos4,"\n")

def detect2(edges):
	i = 0; pos1 = 0; pos2 = 0; pos3 = 0; pos4 = 0;
	while(pos1 == 0):
		pos1 = edges[5,i]; i=i+1
	pos1 = i
	i = 0
	while(pos2 == 0):
		pos2 = edges[5,639-i]; i=i+1
	pos2 = 639-i
	i = 0
	while(pos3 == 0):
		pos3 = edges[463,i]; i=i+1
	pos3 = i
	i = 0
	while(pos4 == 0):
		pos4 = edges[463,639-i]; i=i+1
	pos4 = 639-i
	
	print("cam2:")
	print("   pos1:",pos1," pos2:",pos2," pos3:",pos3," pos4:",pos4,"\n\n\n")
	
while(1):

	img = stream1.read()
	img2 = stream2.read()
	
	img = img[16:480,0:640]
	img2 = img2[16:480,0:640]

	edges = cv.Canny(img,240,255)
	edges2 = cv.Canny(img2,240,255)

	detect1(edges)
	detect2(edges2)

	cv.imshow("imagem 1",edges)
	cv.imshow("imagem 2",edges2)

	if(cv.waitKey(2000)!=-1):
		break
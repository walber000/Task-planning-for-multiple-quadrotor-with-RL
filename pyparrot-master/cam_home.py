from imutils.video import WebcamVideoStream
import numpy as np
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)	
import cv2
import math
import os
from time import sleep

from threading import Thread
import socket
import struct


# Client
ip = '192.168.1.129'
port = 12000
addr = (ip, port)
buff_size = 1024


class client(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.killed = False
		
	def kill(self): 
		self.killed = True
	
	def run(self):
		try:
			while(1):
				global posX_cm, posY_cm, posZ_cm, ang
				ip = '192.168.1.129'
				port = 12000
				addr = (ip, port)
				buff_size = 1024

				client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
				client_socket.settimeout(1.0)
				data = [round(posX_cm,2), round(posY_cm,2), round(posZ_cm,2), round(ang,1)]
				msg = struct.pack('<4f', *data)
				by = client_socket.sendto(msg, addr)
				# sleep(0.03)
				if(self.killed):
					break
		except KeyboardInterrupt:
			print("bye thread..")
			self.killed = True



def detect_cam1(img1, h):
	global xmin, xmax, ymin, ymax
	size_max = 0; i_max = 0
	posX = 0; posY = 0
	posX_cm = 0; posY_cm = 0
	ang = 0;
	cx2 = 0; cy2 = 0
	
	# img1[:,1540:1920,:]=(255, 255, 255)
	
	lower = lower_blue
	upper = upper_blue
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower, upper)
	res = cv2.bitwise_and(img1,img1, mask=mask)
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
		if(M['m00']!=0 and size_max > 40):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img1,0,0,0,0,0
		
		roi = img1[max((cy-30),0):min((cy+30),1080),max((cx-30),0):min((cx+30),1920)]
		lower = lower_ylw
		upper = upper_ylw
		hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, lower, upper)
		res = cv2.bitwise_and(roi,roi, mask=mask)
		imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
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
			if(M['m00']!=0 and size_max > 40):
				cx2 = int(M['m10']/M['m00']) #centroide
				cy2 = int(M['m01']/M['m00'])
				cx2 = cx2 + max((cx-30),0) #ajustes devido à ROI
				cy2 = cy2 + max((cy-30),0)
			else:
				return img1,0,0,0,0,0
			
		else:
			return img1,0,0,0,0,0
	else:
		return img1,0,0,0,0,0
	
	cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	# cv2.drawContours(img1, cnt, -1, (0,255,0), 2)
	# cv2.drawContours(img1, cnt2, -1, (0,255,0), 2)
	#linha / angulo
	cv2.line(img1,(cx,cy),(cx2,cy2),(0,0,255),1)
	ang = math.atan2(-(cy-cy2),-(cx-cx2)) #azul-amarelo = final - inicial #negativo pois eixos da camera sao inverso dos eixos reais
	ang = ang*180/3.14159
	
	#posicao corrigida devido a altura
	posX = (cx+cx2)/2
	posY = (cy+cy2)/2

	x_cam = 160
	y_cam = 230
	
	xlim_min = xmin + (x_cam-xmin)*h/hmax
	xlim_max = xmax - (xmax-x_cam)*h/hmax
	ylim_min = ymin + (y_cam-ymin)*h/hmax
	ylim_max = ymax - (ymax-y_cam)*h/hmax
	
	posX_cm = xlim_min + ((1920-posX)*(xlim_max-xlim_min)/1920)
	posY_cm = ylim_min + ((1080-posY)*(ylim_max-ylim_min)/1080)
	
	return img1, posX, posY, posX_cm, posY_cm, ang

def detect_cam3(img3, y):
	global zmin, zmax
	size_max = 0; i_max = 0; cx2 = 0; cy2 = 0
	# print(img3.shape)
	img3[:,1300:1920,:]=(255, 255, 255)

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
	# posX = cx
	posZ = cy
	h_cam = 140
	d_cam = 459
	lim_min = h_cam + y*(zmin-h_cam)/d_cam
	lim_max = h_cam + y*(zmax-h_cam)/d_cam
	# print("zlim_min: %.1f"%lim_min," , zlim_max: %.1f"%lim_max) 
	posZ_cm =  lim_min + ((1080-posZ)*(lim_max-lim_min)/1080)
	
	return img3, posZ_cm, cx, cy

def cam1_calib(img1):

	size_max = 0; i_max = 0
	cx = 0; cy = 0; cx2 = 0; cy2 = 0
	lower = lower_green
	upper = upper_green
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask1 = cv2.inRange(hsv, lower, upper)
	res = cv2.bitwise_and(img1,img1, mask=mask1)
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
		if(M['m00']!=0 and size_max > 20):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return 0,0,0,0,[0,0,0,0]
	else:
		return 0,0,0,0,[0,0,0,0]
	
	
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
		if(M['m00']!=0 and size_max > 20):
			cx2 = int(M['m10']/M['m00']) #centroide
			cy2 = int(M['m01']/M['m00'])
	
	# print(cx)
	# print(cy)
	# cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	# cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	#posicao corrigida devido a altura
	py1=min(cy,cy2)
	py2=max(cy,cy2)
	px1=max(cx,cx2)
	px2=min(cx,cx2)
	print("px1: %.1f"%px1," , px2: %.1f"%px2) 
	print("py1: %.1f"%py1," , py2: %.1f"%py2) 
	x1 = 61
	x2 = 61
	y1 = 180
	y2 = 218
	py1=1080 - py1
	py2=1080 - py2
	
	ratio = (py1-py2)/(y2-y1)
	xmin = x1 - (1920-px1)/ratio
	xmax = x1 + px1/ratio
	#  = (x2*px2-x1*px1)/(px2-px1) #d = 450cm
	# xmax = ((x2-x1)*(1920-px1)/(px1-px2)) + x2 #d = 450cm
	ymin = (y2*py2-y1*py1)/(py2-py1) #d = 450cm
	ymax = ((y2-y1)*(1080-py1)/(py1-py2)) + y2 #d = 450cm
	
	return xmin, xmax, ymin, ymax, [cx, cx2, cy, cy2]

def cam3_calib(img1):

	size_max = 0; i_max = 0
	cx = 0; cy = 0; cx2 = 0; cy2 = 0;
	lower = lower_green
	upper = upper_green
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask1 = cv2.inRange(hsv, lower, upper)
	res = cv2.bitwise_and(img1,img1, mask=mask1)
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
		if(M['m00']!=0 and size_max > 20):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return 0,0,[0,0,0,0]
	else:
		return 0,0,[0,0,0,0]
	
	
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
		if(M['m00']!=0 and size_max > 20):
			cx2 = int(M['m10']/M['m00']) #centroide
			cy2 = int(M['m01']/M['m00'])
	
	# print(cx)
	# print(cy)
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	#posicao corrigida devido a altura
	p1=min(cy,cy2)
	p2=max(cy,cy2)
	
	p1=1080 - p1
	p2=1080 - p2
	
	
	print("p1: %.1f"%p1," , p2: %.1f"%p2) 
	
	h1 = 11 #11
	h2 = 121 #55
	zmin = (h2*p2-h1*p1)/(p2-p1) #d = 450cm
	zmax = ((h2-h1)*(1080-p1)/(p1-p2)) + h2 #d = 450cm
		
	return zmin, zmax, [cx,cx2,cy,cy2]

def detect_color(img1, h):
	global posX_cm, posY_cm, ang
	size_max = 0; i_max = 0
	cx = 0; cy = 0
	lower = lower_green
	upper = upper_green
	hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
	mask1 = cv2.inRange(hsv, lower, upper)
	res = cv2.bitwise_and(img1,img1, mask=mask1)
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
		if(M['m00']!=0 and size_max > 20):
			cx = int(M['m10']/M['m00']) #centroide
			cy = int(M['m01']/M['m00'])
		else:
			return img1,0,0,0,0
	else:
		return img1,0,0,0,0
	
	
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
		if(M['m00']!=0 and size_max > 20):
			cx2 = int(M['m10']/M['m00']) #centroide
			cy2 = int(M['m01']/M['m00'])
	
	print(cx)
	print(cy)
	cv2.circle(img1,(cx,cy),2,(0,0,255),2)
	cv2.circle(img1,(cx2,cy2),2,(0,0,255),2)
	#posicao corrigida devido a altura
	posX = cx
	posY = cy
	ymin = 39 #30 #33
	ymax = 176 #171.5
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (90-ymin)*h/hmax
	ylim_max = ymax - (ymax-90)*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/1920)
	posY_cm = ylim_min + ((1080-posY)*(ylim_max-ylim_min)/1080)
	
	return img1, posX, posY, posX_cm, posY_cm

lower_blue = np.array([84,37,143]) #old = 84 60 80 / 84 42 145
upper_blue = np.array([132,255,255]) 

# lower_blue = np.array([84,12,80]) #eug
# upper_blue = np.array([132,255,255]) 

lower_green = np.array([48,40,70]) # 48 70 70
upper_green = np.array([90,255,255]) # 75/80 255 255 

lower_ylw = np.array([23,100,140]) 
upper_ylw = np.array([45,255,255]) #45 240 250

# lower_ylw = np.array([23,32,140]) #eug
# upper_ylw = np.array([45,255,255]) #45 240 250

# lower_red1 = np.array([0,100,50]) 
# upper_red1 = np.array([11,255,255])

# lower_red2 = np.array([165,40,0])  #164 100 50
# upper_red2 = np.array([180,255,255])

lower_red1 = np.array([160,63,70]) # 160 70 70
upper_red1 = np.array([180,255,255])
lower_red2 = np.array([160,63,70]) 
upper_red2 = np.array([180,255,255])


# url1 = "http://192.168.1.105:8080/video" #eug
url1 = "http://192.168.1.114:8080/video" #dan
url3 = "http://192.168.1.106:8080/video" #me

posX_cm = 0; posY_cm = 0; posZ_cm = 0 ; ang = 0
X = []; Y = []
h = 0; hmax = 200
stream1 = WebcamVideoStream(src=url1).start()
stream3 = WebcamVideoStream(src=url3).start()

savefile = 1
savevideo = 0

# SAVE DATA IN TXT
if(savefile):
	X=[];Y=[];Z=[]
	i=1
	name = "data_saved%d.txt"%i
	while(os.path.isfile(name)):
		i=i+1
		name = "data_saved%d.txt"%i
	f = open(name, 'w')
	print("opening %s..."%name)

# SAVE VIDEO
# if(savevideo):
	# cap = cv2.VideoCapture(url1)
	# cap2 = cv2.VideoCapture(url3)
	# fourcc = cv2.VideoWriter_fourcc(*'DIVX')
	# out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1920,1080))
	# out2 = cv2.VideoWriter('output2.avi',fourcc, 20.0, (1920,1080))

thread1 = client()
thread1.start()

try:
	# sleep(8)
	img1 = stream1.read()
	img3 = stream3.read()
	xmin, xmax, ymin, ymax, [cx_xy, cx2_xy, cy_xy, cy2_xy] = cam1_calib(img1)
	zmin, zmax, [cx_z, cx2_z, cy_z, cy2_z] = cam3_calib(img3)
	cv2.namedWindow('Resized Window1', cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Resized Window1', 800, 600)
	cv2.namedWindow('Resized Window3', cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Resized Window3', 800, 600)
	print("xmin: %.1f"%xmin," , xmax: %.1f"%xmax)
	print("ymin: %.1f"%ymin," , ymax: %.1f"%ymax)
	print("zmin: %.1f"%zmin," , zmax: %.1f"%zmax)
	# cv2.imshow("Resized Window",img1)
	# sleep(2)
	while(1):
		img1 = stream1.read()
		img3 = stream3.read()
		
		# posZ_cm = 40
		img1, posX, posY, posX_cm, posY_cm, ang = detect_cam1(img1,posZ_cm)
		if(posX_cm != 0):
			img3, posZ_cm, cx, cy = detect_cam3(img3,posY_cm) 
		print("X: %.1f"%posX_cm,"Y: %.1f"%posY_cm, "Z: %.1f"%posZ_cm, "ang: %.1f"%ang)  
		# posZ_cm = 40
		# zmin, zmax = cam3_calib(img3)
		
		cv2.circle(img1,(cx_xy,cy_xy),2,(0,0,255),2)
		cv2.circle(img1,(cx2_xy,cy2_xy),2,(0,0,255),2)
		cv2.circle(img3,(cx_z,cy_z),2,(0,0,255),2)
		cv2.circle(img3,(cx2_z,cy2_z),2,(0,0,255),2)
		
		cv2.imshow("Resized Window1",img1)
		cv2.imshow("Resized Window3",img3)
		# print("zmin: %.1f"%zmin," , zmax: %.1f"%zmax)  
		# sleep(1)
		if(savefile):
			X.append(posX_cm)
			Y.append(posY_cm)
			Z.append(posZ_cm)			

		# if(savevideo):
			# ret, frame = cap.read()
			# out.write(frame)
			# ret, frame = cap2.read()
			# out2.write(frame)

		if(cv2.waitKey(1)!=-1):
			thread1.kill()  
			break
				
except Exception or KeyboardInterrupt as e:
	print("abortado: ", e)
	thread1.kill() 
	#sys.exc_info()[0]

if(savefile):
	print("saving...")
	for item in range(len(Z)):		
		f.write("%.1f," % X[item])
		f.write("%.1f," % Y[item])
		f.write("%.1f\r\n" % Z[item])
	print("saved!")
	f.close()

if(savevideo):
	# cap.release()
	# cap2.release()
	# out.release()
	# out2.release()
	cv2.destroyAllWindows()

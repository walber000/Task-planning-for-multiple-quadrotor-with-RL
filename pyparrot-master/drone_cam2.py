from __future__ import print_function

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)	
import cv2

from imutils.video import WebcamVideoStream
import imutils
import numpy as np
from time import sleep
from threading import Thread
import math
from tkinter import *
from coursework.droneMapGUI import DroneGUI
from pyparrot.Minidrone import Mambo

def bias_roll(posx):
	roll_b = 5
	global posX_cm
	# if(mambo.sensors.speed_y<-0.01):
		# roll_b = 15
	# if(mambo.sensors.speed_y>0.01):
		# roll_b = -10
	if(posX_cm<(-8+posx)):
		roll_b = -10 #drone flipado em 180g
	if(posX_cm>(8+posx)):
		roll_b = 18 #drone flipado em 180g
	return roll_b

def correct_yaw(base,tol):
	global ang
	count = 0
	print("corrigindo yaw...")
	while(ang==0):
		count = count + 1
		if(count>12):			
			print("count exceeded: ang = 0")
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			kill_thread = 1
			break
	count = 0
	if(ang>base+tol):
		while (ang>base+tol) or (ang == 0):
			if(ang == 0):
				count = count + 1
				if(count > 12):
					print("count exceeded: ang = 0")
					mambo.safe_land(5)
					mambo.smart_sleep(1)
					print("disconnect")
					mambo.disconnect()
					kill_thread = 1
					break
			else:
				mambo.fly_direct(roll=0, pitch=0, yaw=-6, vertical_movement=0, duration=0.1)
				count = 0
			print("yaw: ", ang)
	if(ang<base-tol):
		while (ang<base-tol) or (ang == 0):
			if(ang == 0):
				count = count + 1
				if(count > 12):
					print("count exceeded: ang = 0")
					mambo.safe_land(5)
					mambo.smart_sleep(1)
					print("disconnect")
					mambo.disconnect()
					kill_thread = 1
					break
			else:
				mambo.fly_direct(roll=0, pitch=0, yaw=6, vertical_movement=0, duration=0.1)
				count = 0
			print("yaw: ", ang)
	print("done adjusting yaw!")
	mambo.smart_sleep(2)
	print("new yaw: ", ang)
	return

def controleX(x_d,tol):
	print("Controle no eixo X...")
	posy = posY_cm
	erro_x = x_d - posX_cm
	while(posX_cm == 0):
		posy = posY_cm
		erro_x = x_d - posX_cm
		
	pitch = 0
	count = 0

	while (abs(erro_x)>tol or posX_cm == 0):
		if(posX_cm != 0):
			count = 0
			roll = bias_roll(posy)
			erro_x = x_d - posX_cm
			if(abs(erro_x) > 3*tol):
				pitch = 10
			else:	
				pitch = -1
				print("slowing...")
			
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x, \
			" pitch: ", pitch, " roll: ", roll)
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.1)
			#mambo.smart_sleep(0.1)
		else:
			print("camera missed")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			kill_thread = 1
			break

def controleY(y_d,tol):
	print("Controle no eixo Y...")
	posx = posX_cm
	erro_y = y_d - posY_cm
	while(posX_cm == 0):
		posx = posX_cm
		erro_y = y_d - posY_cm
	pitch = 0
	count = 0
	
	print("entrando while...")
	while ((abs(erro_y)>tol) or (posX_cm == 0)):
		print("4")
		if(posX_cm != 0):
			count = 0
			roll = bias_roll(posx)
			erro_y = y_d - posY_cm
			if(abs(erro_y) > 3*tol):
				pitch = 10
			else:	
				pitch = -1
				print("slowing...")
			
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_y: %.2f" % erro_y, \
			" pitch: ", pitch, " roll: ", roll, "vel_y: %.1f" %mambo.sensors.speed_y)
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.1)
			#mambo.smart_sleep(0.1)
		else:
			print("camera missed")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			kill_thread = 1
			break

def altitude(h):
	count = 0
	print("going to altitude %.1f centimeters..." % h)
	while ((posZ_cm>h or posX_cm == 0) and count < 3):
		if(kill_thread==1):
			break
		if(posZ_cm<=h):
			count = count + 1
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
		else:
			count = 0
			if(posX_cm == 0):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			else:
				if(posZ_cm<h+10):
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-4, duration=0.1)
				else:
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-6, duration=0.1)
		print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
	print("done!\n")
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	mambo.smart_sleep(2)

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
			#gui.root.update_idletasks()
			gui.root.update()
			sleep(0.05)
			gui.position(posX_cm*2,560-posY_cm*2)

class drone(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self):
		try:
			global kill_thread, posX_cm, posY_cm, posZ_cm, ang, mambo
			
			mambo.ask_for_state_update()
			mambo.smart_sleep(0.5)
			
			print("taking off!")
			mambo.safe_takeoff(5)
			mambo.smart_sleep(3)
			
			max_tilt = 10
			mambo.set_max_tilt(max_tilt)

			if(kill_thread==0):
				correct_yaw(-90,4)
			
			if(kill_thread==0):
				altitude(40)
				
			if(kill_thread==0):
				correct_yaw(-90,4)
			
			# #WAYPOINT
			x_d = 80
			y_d = 80
			tol = 5 #tolerancia de erro em cm
			
			if(kill_thread==0):
				controleY(y_d,tol)
			
			if(kill_thread==0):
				print("arrived pos_y\n")
				mambo.smart_sleep(2)
				print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
				correct_yaw(-180,4)
			
			if(kill_thread==0):
				controleX(x_d,tol)
				
			if(kill_thread==0):
				print("arrived pos_x\n")
				mambo.smart_sleep(2)
				print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
				
		except:
			print("abortado")
			
		print("landing")
		mambo.safe_land(5)
		mambo.smart_sleep(1)
		
		print("disconnect")
		mambo.disconnect()
		kill_thread = 1

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
	ymin = 35
	ymax = 175
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
	ymin = 102
	ymax = (279-33)
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (226-ymin)*h/hmax
	ylim_max = ymax - (ymax-226)*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm")
	
	return img2, posX_cm, posY_cm, ang

def detect_cam3(img3, y):
	global sizemin, sizemax, posZ_cm
	size_max = 0; size_max2 = 0
	i_max = 0; i_max2 = 0
	
	M3 = cv2.getPerspectiveTransform(pts4,pts3)
	img3 = cv2.warpPerspective(img3,M3,(640,480))
	
	img3 = cv2.undistort(img3, mtx, dist)
	
	img3 = img3[0:420,160:430]

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
	if(M['m00']!=0):
		res = res[max(cy-50,0):min(cy+50,480),max(cx-50,0):min(cx+50,640)]
		#print(max(cy-50,0)," ",min(cy+50,480)," ",max(cx-50,0)," ",min(cx+50,640))
		imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(imgray, 40, 255, 0)
		contours, hierarchy = cv2.findContours(thresh, 1, 2)
	#del contours[i_max]
	if(len(contours)>0):
		for i in range(len(contours)):
			cnt2 = contours[i]
			size = cv2.contourArea(cnt2)
			if (size > size_max2)and(size !=size_max):
				size_max2 = size
				i_max2 = i
	
		cnt2=contours[i_max2]
		M = cv2.moments(cnt2)
		if(M['m00']!=0):
			cx2 = int(M['m10']/M['m00']) + (cx-50) #centroide
			cy2 = int(M['m01']/M['m00']) + (cy-50)
	
	#print(size_max," ",size_max2)
	
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
		#cv2.drawContours(img3, cnt2, -1, (0,255,0), 2)
		cx = (cx+cx2)/2; cy = (cy+cy2)/2
	
	#posicao corrigida devido a altura
	y = y + 178
	posX = cx ; posZ = cy
	zlim_min = 34.5 - 50*y/178
	zlim_max = 34.5 + 58*y/178
	posZ_cm =  zlim_min + ((480-posZ)*(zlim_max-zlim_min)/480)
	
	return img3, posZ_cm
	
#inicializacao de variaveis
posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0; kill_thread = 0
sizemin = 330; sizemax = 0
cx = 0; cy = 0
cx2 = 0; cy2 = 0
i = 0; i2 = 0; i3 = 0
#h = 24.5 #cone = 24.2, tampa = 0.3
hmax = 246

#thread = GUI()
#thread.start()

mtx = np.array([[845, 0, 319.6],[0, 891, 157.3], [0, 0, 1]])
dist = np.array([[ 0.265530618, -2.45263550, -0.0382963307,
-0.000583787615,   6.56047640]])

pts1 = np.float32([[32,16],[567,16],[583,480],[34,480]]) #cam1
pts2 = np.float32([[32,16],[597,16],[570,480],[78,480]]) #cam2
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
	print("deu ruim nas cameras")
	kill_thread = 1
	sleep(1)
	good = 0

M1 = cv2.getPerspectiveTransform(pts1,pts3)
M2 = cv2.getPerspectiveTransform(pts2,pts3)
M3 = cv2.getPerspectiveTransform(pts4,pts3)

lower_blue = np.array([104,80,40]) 
upper_blue = np.array([131,250,250]) 

lower_ylw = np.array([23,100,140]) 
upper_ylw = np.array([45,240,250])

lower_red1 = np.array([0,50,50]) 
upper_red1 = np.array([10,255,255])

lower_red2 = np.array([172,50,50]) 
upper_red2 = np.array([179,255,255])
	
mamboAddr = "D0:3A:5E:20:E6:21"
mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)
if(success):
	thread2 = drone()
	thread2.start()

while(good==1):
	#le imagens
	try:
		img1 = stream1.read()
		img2 = stream2.read()
		img3 = stream3.read()

		img1, posX_cm, posY_cm, ang = detect_cam1(img1,posZ_cm)
		if((posX_cm and posY_cm and ang) == 0):
			img2, posX_cm, posY_cm, ang = detect_cam2(img2,posZ_cm)
		else:
			img2 = cv2.warpPerspective(img2,M2,(640,480))
		if((posX_cm or posY_cm or ang) != 0):
			img3, posZ_cm = detect_cam3(img3,posY_cm)
			#print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm   ang = %.1f" % ang)
		else:
			img3 = cv2.warpPerspective(img3,M3,(640,480))
			img3 = cv2.undistort(img3, mtx, dist)

		cv2.imshow("imagem 1",img1)
		cv2.imshow("imagem 2",img2)
		cv2.imshow("imagem 3",img3)
		if(cv2.waitKey(5)!=-1 or kill_thread==1):
			#print("sizemin: ",sizemin," sizemax: ",sizemax)
			print("end main")
			#thread2.join()
			kill_thread = 1
			sleep(3)
			break
	except:
		print("abortado main")
		kill_thread = 1
		#thread2.join()
		sleep(3)
		good = 0
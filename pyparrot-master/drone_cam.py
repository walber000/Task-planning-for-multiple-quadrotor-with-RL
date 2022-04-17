from __future__ import print_function
from imutils.video import WebcamVideoStream
import imutils
import numpy as np
import cv2
from time import sleep
from threading import Thread
import math
from tkinter import *
from coursework.droneMapGUI import DroneGUI
from pyparrot.Minidrone import Mambo

#correcao de bias para drone 1
def bias_roll(posy):
	roll_b = 5
	global posX_cm
	# if(mambo.sensors.speed_y<-0.01):
		# roll_b = 15
	# if(mambo.sensors.speed_y>0.01):
		# roll_b = -10
	if(posX_cm<(-8+posy)):
		roll_b = -10 #drone flipado em 180g
	if(posX_cm>(8+posy)):
		roll_b = 18 #drone flipado em 180g
	return roll_b

#correcao de yaw
def correct_yaw(base):
	(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
	mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
	print("corrigindo yaw...")
	print("psi: ", mambo.sensors.psi)
	if(Z>1+base):
		while Z>1+base:
			mambo.fly_direct(roll=0, pitch=0, yaw=-2, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("yaw: ", Z)
	if(Z<-1+base):
		while Z<-1+base:
			mambo.fly_direct(roll=0, pitch=0, yaw=2, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("yaw: ", Z)
	print("done adjusting yaw!")
	mambo.smart_sleep(2)
	print("new yaw: ", Z)
	print("psi: ", mambo.sensors.psi)
	return

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
			
posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0; kill_thread = 0
#thread = GUI()
#thread.start()

class drone(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self):
		try:
			global kill_thread, posX_cm, posY_cm, posZ_cm, mambo
			count = 0
			
			mambo.ask_for_state_update()
			mambo.smart_sleep(0.5)
			
			print("taking off!")
			mambo.safe_takeoff(5)
			mambo.smart_sleep(3)
			
			max_tilt = 10
			mambo.set_max_tilt(max_tilt)
			
			posy = posX_cm
			
			correct_yaw(0)
			print("ang: %.1f" %ang)
			
			print("going to altitude 0.4 meters...")
			while posZ_cm>40:
				if(kill_thread==1):
					break
				if(posZ_cm<50):
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-3, duration=0.4)
				else:
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-6, duration=0.4)
				print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
			print("done!\n")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
			mambo.smart_sleep(2)
			
			correct_yaw(0)
			print("ang: %.1f" %ang)
			
			if(kill_thread==0):
				mambo.fly_direct(roll=5, pitch=10, yaw=0, vertical_movement=0, duration=2)
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=2)
			
			if(kill_thread==0):
				correct_yaw(90)
				print("ang: %.1f" %ang)
			
			# print("moving forward...\n")
			# while(((posY_cm > 80) or (posY_cm == 0)) and (kill_thread==0)):
				# if(posY_cm == 0):
					# print("camera missed")
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
					# count = count + 1
				# else:
					# count = 0
					# rollb = bias_roll(posy)
					# print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm    rollb = %.1f" %rollb)
					# mambo.fly_direct(roll=rollb, pitch=7, yaw=0, vertical_movement=0, duration=0.1)
				# if(count > 12):
					# print("count exceded! landing...")
					# count = 0
					# mambo.safe_land(5)
					# mambo.smart_sleep(1)
					# print("disconnect")
					# mambo.disconnect()
					# kill_thread = 1
					
			# print("\ndone!")
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
			# print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
			# mambo.smart_sleep(3)
			
			# print("moving back...\n")
			# while(((posY_cm < 180) or (posY_cm == 0)) and (kill_thread==0)):
				# if(posY_cm == 0):
					# print("camera missed")
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
					# count = count + 1
				# else:
					# count = 0
					# rollb = bias_roll(posy)
					# print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm    rollb = %.1f" %rollb)
					# mambo.fly_direct(roll=rollb, pitch=-15, yaw=0, vertical_movement=0, duration=0.1)
				# if(count > 12):
					# print("count exceded! landing...")
					# count = 0
					# mambo.safe_land(5)
					# mambo.smart_sleep(1)
					# print("disconnect")
					# mambo.disconnect()
					# kill_thread = 1
					
			# print("\ndone! kill_thread = %.1f" %kill_thread)
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
			# print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
			# mambo.smart_sleep(2)
			
			# mambo.smart_sleep(3)
			# mambo.fly_direct(roll=7, pitch=-10, yaw=0, vertical_movement=0, duration=3)
			# mambo.smart_sleep(3)
			# mambo.fly_direct(roll=4, pitch=10, yaw=0, vertical_movement=0, duration=3)
			# mambo.smart_sleep(3)
			# mambo.fly_direct(roll=7, pitch=-10, yaw=0, vertical_movement=0, duration=3)
			# mambo.smart_sleep(3)
		
			# #WAYPOINT
			# x_d = -40
			# y_d = -30
			# tol = 8 #tolerancia de erro em cm
			
			# #CONTROLE EM X:
			# print("Controle no eixo X...")
			# erro_x = x_d - mambo.sensors.posx
			# pitch = 0
			# bias = 5 #bias, movimento tende para esquerda

			# while abs(erro_x)>tol:
				# roll = bias_roll() + bias
				# erro_x = x_d - mambo.sensors.posx
				# speed_x = mambo.sensors.speed_x
				# speed_y = mambo.sensors.speed_y
				# if((erro_x/abs(erro_x))*(speed_x/abs(speed_x))!=1): #esta indo em direcao ao aumento do erro
					# pitch = 20*(erro_x/abs(erro_x))
				# else:
					# if(abs(speed_x) < abs(erro_x)/300):
						# pitch = 8*(erro_x/abs(erro_x))
					# else:	
						# pitch = -1
						# print("slowing...")
				
				# print("posx: %.2f" % mambo.sensors.posx , " posy: %.2f" % mambo.sensors.posy, " erro_x: %.2f" % erro_x, \
				# " pitch: ", pitch, " speed_x: %.2f" % speed_x, " roll: ", roll, " speed_y: %.2f" % speed_y)
				# #print(" ")
				# mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.3)
				# mambo.smart_sleep(0.2)
			
			# print("arrived pos_x")
			# print("")
			# mambo.smart_sleep(2)
					
			# print("altitude_cam: %.3f\n" % posZ_cm)
			# mambo.smart_sleep(0.5)
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
	ymin = 30
	ymax = 171.5
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
	ymin = 126.3
	ymax = (279-11)
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
	
	img3 = img3[0:420,150:430]

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
	zlim_max = 34.5 + 57*y/178
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
		if(cv2.waitKey(50)!=-1 or kill_thread==1):
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
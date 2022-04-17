#!/usr/bin/env python3

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)	
import cv2
from imutils.video import WebcamVideoStream
from imutils.video import FPS
#import argparse
import imutils
import numpy as np
import time
from time import sleep
#import datetime
from threading import Thread
import math
from __future__ import print_function
from pyparrot.Minidrone import Mambo

posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
posX2_cm = 0; posY2_cm = 0; posZ2_cm = 0; ang2 = 0
posX3_cm = 0; posY3_cm = 0; posZ3_cm = 0
size_max_thresh = 80; size_max_thresh_h = 15

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
		if(M['m00']!=0 and size_max > size_max_thresh):
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
			if(M['m00']!=0 and size_max > size_max_thresh):
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
	ymin = 39 #30 #33
	ymax = 176 #171.5
	xlim_min = 0 + 92*h/hmax
	xlim_max = 164 - 72*h/hmax
	ylim_min = ymin + (90-ymin)*h/hmax
	ylim_max = ymax - (ymax-90)*h/hmax
	posX_cm = xlim_min + (posX*(xlim_max-xlim_min)/640)
	posY_cm = ylim_min + ((480-posY)*(ylim_max-ylim_min)/480)
	
	return img1, posX_cm, posY_cm, ang
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
		if(M['m00']!=0 and size_max > size_max_thresh):
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
			if(M['m00']!=0 and size_max > size_max_thresh):
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
		if(M['m00']!=0 and size_max > size_max_thresh_h):
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
		if(M['m00']!=0 and size_max > size_max_thresh_h):
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
			
		except:
			print("abortado")
			
		print("landing")
		mambo.safe_land(5)
		mambo.smart_sleep(1)
		
		print("disconnect")
		mambo.disconnect()
		kill_thread = 1



#inicializacao de variaveis
# cx = 0; cy = 0
# cx2 = 0; cy2 = 0
# i = 0
hmax = 246

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

lower_blue = np.array([84,42,80]) 
upper_blue = np.array([132,255,255]) 

lower_green = np.array([48,70,70]) 
upper_green = np.array([75,255,255])

lower_ylw = np.array([23,100,140]) 
upper_ylw = np.array([45,240,250])

lower_red1 = np.array([0,90,30]) 
upper_red1 = np.array([11,255,255])

lower_red2 = np.array([165,40,0])  #164 100 50
upper_red2 = np.array([180,255,255])

lower_pink1 = np.array([0,35,150]) 
upper_pink1 = np.array([11,130,255])

lower_pink2 = np.array([140,35,150]) 
upper_pink2 = np.array([180,130,255])

thread2 = cam()
thread2.start()



while(good==1):
	try:
		#le imagens
		img1 = stream1.read()
		img2 = stream2.read()
		img3 = stream3.read()
		
		# drone 1
		img1, posX_cm, posY_cm, ang = detect_cam1(img1,posZ_cm)
		if(posX_cm == 0):
			img2, posX_cm, posY_cm, ang = detect_cam2(img2,posZ_cm)
		if(posX_cm != 0):
			img3, posZ_cm, cx, cy = detect_cam3(img3,posY_cm)		

			pos = [posX_cm,posY_cm,posZ_cm,ang,posX2_cm,posY2_cm,posZ2_cm,ang2,posX3_cm,posY3_cm,posZ3_cm]
			pos = Float32MultiArray(data=pos)
			rospy.loginfo("drone1: %.2f" % pos.data[0] + " %.2f" % pos.data[1] + " %.2f" % pos.data[2] + " %.1f", pos.data[3])
			pub.publish(pos)
			rate.sleep()


	except:
		break









def bias_roll():
	global roll_base
	roll_b = roll_base
	if(mambo.sensors.speed_y<-0.015):
		#roll_b = roll_b*0.85
		roll_b = 5 + roll_base
	if(mambo.sensors.speed_y>0.015):
		#roll_b = roll_b*1.15
		roll_b = -5 + roll_base
	if(mambo.sensors.speed_y<-0.03):
		#roll_b = roll_b*0.85
		roll_b = 10 + roll_base
	if(mambo.sensors.speed_y>0.03):
		#roll_b = roll_b*1.15
		roll_b = -10 + roll_base

	return roll_b

def bias_roll2(posx, dir): #dir=1 -> X, Y+ / dir=-1 -> X, Y- / dir=2 -> Y, X+ / dir=-2 -> Y, X-
	global posX_cm, posY_cm, roll_base, ang, tol_ang, tol_yaw
	tol = 10
	roll = 0
	vel = 15
	pitch = 0
	pitch_c = 10
	
	if dir == 1 or dir == -1:
		pos = posX_cm
		pos2 = posY_cm
	elif dir == 2 or dir == -2:
		pos = posY_cm
		pos2 = posX_cm
	
	if(abs(dir)==2):
		dir = -dir/2
	
	while(pos<(posx-tol) or pos>(posx+tol)):
		#if(not(abs(ang)<tol_ang or abs(ang)>(180-tol_ang))): 
		#	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
		#	correct_yaw((45*dir + 90),tol_yaw) 
		if(pos<(posx-tol)):
			roll = roll_base + vel*dir
			print("roll: ", roll)
			print("pitch: ", pitch)
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=1)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
			mambo.flat_trim()
		if(pos>(posx+tol)):
			roll = roll_base -vel*dir
			print("roll: ", roll)
			print("pitch: ", pitch)
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=1)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
			mambo.flat_trim()
		if dir == 1 or dir == -1:
			pos = posX_cm
			if(posY_cm<pos2-5):
				pitch = pitch_c*dir
			elif(posY_cm>pos2+5):
				pitch = -pitch_c*dir
		elif dir == 2 or dir == -2:
			pos = posY_cm
			if(posX_cm<pos2-5):
				pitch = pitch_c*dir
			elif(posX_cm>pos2+5):
				pitch = -pitch_c*dir
			
	if(roll!=0):
		roll_base = roll_base + math.copysign(1,roll)
	return roll_base
	
def bias_roll3(posx, posy, ang_d): #posx e posy sao posicao inicial do waypoint de onde o drone saiu, ang eh o angulo desejado para aquela trajetoria saindo do wp, nao usar 90graus
	global posX_cm, posY_cm, roll_base, pitch_base
	roll = 0
	vel = 20
	tol = 10 #drone deve se manter dentro de um canal de tol cm para cada lado, enquanto segue a linha central (mov desejado) de angulo ang
	#ang_d = -ang_d #angulo que será usado para calculos é diferente do angulo calculado pela camera
	ang_rad = -ang_d*math.pi/180

	dx = posX_cm - posx
	dy = posY_cm - posy
	
	dy_esperado = dx * math.tan(ang_rad)
	dy_max = tol*math.sqrt((1+math.tan(ang_rad)**2))

	print("dy: %.2f"%dy,", range dy: %.2f" % (dy_esperado-dy_max),"/%.2f"%(dy_esperado+dy_max))
	if(abs(ang_d)<90):
		sign = 1
	else:
		sign = -1
		
	erro = (dy_esperado - dy)*sign
	K = 1
	roll_b = K*erro + roll_base
	
	if((mambo.sensors.speed_y<-0.03 and roll_b < roll_base) or (mambo.sensors.speed_y>0.03 and roll_b > roll_base)):
		roll_b = roll_b*0.85
	if((mambo.sensors.speed_y<-0.03 and roll_b > roll_base) or (mambo.sensors.speed_y>0.03 and roll_b < roll_base)):
		roll_b = roll_b*1.15
		
	if(roll_b>30):
		roll_b=30
	elif(roll_b<-30):
		roll_b=-30
		
	# while((dy<(dy_esperado-dy_max)) or (dy>(dy_esperado+dy_max))):
		# if(abs(ang_d - ang)>tol_ang):
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
			# correct_yaw(ang_d,tol_yaw) 
		# if(dy<(dy_esperado-dy_max)):
			# roll = roll_base + math.copysign(vel,sign)
			# print("roll: ", roll)
			# mambo.fly_direct(roll=roll, pitch=pitch_base, yaw=0, vertical_movement=0, duration=1)
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
			# mambo.flat_trim()
		# if(dy>(dy_esperado+dy_max)):
			# roll = roll_base + math.copysign(vel,-sign)
			# print("roll: ", roll)
			# mambo.fly_direct(roll=roll, pitch=pitch_base, yaw=0, vertical_movement=0, duration=1)
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
			# mambo.flat_trim()
		# dx = posX_cm - posx
		# dy = posY_cm - posy
		# dy_esperado = dx * math.tan(ang_rad)
		# dy_max = tol*math.sqrt((1+math.tan(ang_rad)**2))
		# print("dy: %.2f"%dy,", range dy: %.2f" % (dy_esperado-dy_max),"/%.2f"%(dy_esperado+dy_max))
	# if(roll!=0):
		# roll_base = roll_base + math.copysign(1, roll)
	return roll_b

def correct_yaw(base,tol):
	global ang, posX_cm
	count = 0
	vel = 15
	print("corrigindo yaw...")
	while(ang==0 and posX_cm == 0):
		count = count + 1
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
		print("camera miss...")
		if(count>12):			
			print("count exceeded: ang = 0")
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			return
	count = 0
	
	# if(ang>base+tol):
	while (abs(ang-base)>tol) or (ang == 0):
		if(ang == 0 and posX_cm == 0):
			count = count + 1
			if(count > 12):
				print("count exceeded: ang = 0")
				mambo.safe_land(5)
				mambo.smart_sleep(1)
				print("disconnect")
				mambo.disconnect()
				return
		else:
			if(ang-base>0):
				if(ang-base>180):
					mambo.fly_direct(roll=0, pitch=0, yaw=vel, vertical_movement=0, duration=0.1)
				else:
					mambo.fly_direct(roll=0, pitch=0, yaw=-vel, vertical_movement=0, duration=0.1)
			if(ang-base<0):
				if(ang-base<-180):
					mambo.fly_direct(roll=0, pitch=0, yaw=-vel, vertical_movement=0, duration=0.1)
				else:
					mambo.fly_direct(roll=0, pitch=0, yaw=vel, vertical_movement=0, duration=0.1)
			count = 0
		print("yaw: %.1f" % ang)
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
	print("done adjusting yaw!")
	mambo.smart_sleep(0.5)
	print("new yaw: %.1f\n" % ang)
	return

def controleX(x_d,tol):
	global roll_base, pitch_base
	print("Controle no eixo X...")
	posy = posY_cm
	posx = posX_cm
	erro_x = x_d - posX_cm
	
	while(posX_cm == 0):
		posy = posY_cm
		erro_x = x_d - posX_cm
	
	pitch = 0
	roll = 0
	count = 0
	vel = 15
	vel_fino = 40

	if(erro_x<0):
		dir = -2
		ang_d = 0
	else:
		dir = 2
		ang_d = 180
	while (((abs(erro_x)>tol) and (erro_x/dir > 0)) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			#roll_b = bias_roll2(posy, dir)
			roll_b = bias_roll()
			
			if(not(abs(ang)<tol_ang or abs(ang)>(180-tol_ang))): 
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				correct_yaw((ang_d),tol_yaw) 
			
			if(abs(mambo.sensors.speed_x) > 0.23):
				pitch = 0
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				mambo.flat_trim()
				print("slowing...")
			elif(abs(erro_x) < 15):
				if(abs(mambo.sensors.speed_x) > 0.09):
					pitch = 0
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
					mambo.flat_trim()
					print("slowing close...")
				else:
					pitch = math.copysign(1,erro_x/dir)*vel
			else:
				pitch = math.copysign(1,erro_x/dir)*vel
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x, \
			" pitch: ", pitch, " roll: ", roll_b, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
			mambo.fly_direct(roll=roll_b, pitch=pitch+pitch_base, yaw=0, vertical_movement=0, duration=0.1)
			erro_x = x_d - posX_cm
			#mambo.smart_sleep(0.1)
		else:
			print("camera missed")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			break
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	while(abs(x_d-posX_cm)>tol):
		pitch = math.copysign(vel_fino,dir*(x_d-posX_cm))
		mambo.fly_direct(roll=roll_base, pitch=pitch+pitch_base, yaw=0, vertical_movement=0, duration=0.2)
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)

def controleXroll(x_d,tol):
	print("Controle no eixo X...")
	erro_x = x_d - posX_cm
	while(posX_cm == 0):
		erro_x = x_d - posX_cm
		
	roll = 0
	vel_x = 15
	vel_x_fino = 40
	count = 0
		
	while ((abs(erro_x)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			if(abs(ang)<75 or abs(ang)>105):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				correct_yaw(math.copysign(90,ang),tol_yaw)
			if(abs(mambo.sensors.speed_y) > 0.23):
				roll = 0
				print("slowing...")
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				mambo.flat_trim()
			elif(abs(erro_x) < 18):
				if(abs(mambo.sensors.speed_y) > 0.1):
					roll = 0
					print("slowing close...")
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
					mambo.flat_trim()
				else:
					roll = math.copysign(vel_x,erro_x*ang)
			else:
				roll = math.copysign(vel_x,erro_x*ang)
				
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x, \
			" roll: ", roll, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
			mambo.fly_direct(roll=roll, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
			erro_x = x_d - posX_cm
			#mambo.smart_sleep(0.1)
		else:
			print("camera missed")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			break
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	correct_yaw(math.copysign(90,ang),tol_yaw)
	print("Controle fino em X (roll)...")
	while(abs(x_d-posX_cm)>tol):
			roll = math.copysign(vel_x_fino,ang*(x_d-posX_cm))
			mambo.fly_direct(roll=roll, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	print("done!\n")
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)

def controleY(y_d,tol):
	global roll_base, pitch_base
	print("Controle no eixo Y...")
	posx = posX_cm
	erro_y = y_d - posY_cm
	while(posX_cm == 0):
		posx = posX_cm
		erro_y = y_d - posY_cm

	pitch = 0
	roll = 0
	count = 0
	vel_y = 15
	vel_y_fino = 40
	# range_bias = 7
	
	if(erro_y<0):
		ang_d = -90
	else:
		ang_d = 90
	
	while ((abs(erro_y)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0

			#roll_b = bias_roll2(posx, math.copysign(1,ang))
			#roll_b = bias_roll3(posx, posy, ang_d)
			roll_b = bias_roll()
			
			#bias_roll2 implementado na propria funçao controleY
			# if(posX_cm<(-range_bias+posx) or posX_cm>(range_bias+posx)):
				# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
				# while(posX_cm<(-range_bias+posx) or posX_cm>(range_bias+posx)):
					# if(abs(ang)<(90-tol_ang) or abs(ang)>(90+tol_ang)):
						# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
						# correct_yaw(math.copysign(90,ang),tol_yaw)
					# bias = math.copysign(25,(posX_cm-posx)*(-erro_y))
					# print("bias: ", bias)
					# mambo.fly_direct(roll=bias, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
				# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
				# roll = roll + math.copysign(1,bias)
				# mambo.flat_trim()
			#end bias roll
			
			if(abs(ang)<(90-tol_ang) or abs(ang)>(90+tol_ang)):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				correct_yaw(math.copysign(90,ang),tol_yaw)
			if(abs(mambo.sensors.speed_x) > 0.3):
				pitch = 0
				print("slowing...")
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				mambo.flat_trim()
			elif(abs(erro_y) < 20):
				if(abs(mambo.sensors.speed_x) > 0.13):
					pitch = 0
					print("slowing close...")
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
					mambo.flat_trim()
				else:
					pitch = math.copysign(vel_y,erro_y*ang)
			else:
				pitch = math.copysign(vel_y,erro_y*ang)
			mambo.fly_direct(roll=roll_b, pitch=pitch+pitch_base, yaw=0, vertical_movement=0, duration=0.1)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_y: %.2f" % erro_y, \
			" pitch: ", pitch, " roll: ", roll_b, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
			erro_y = y_d - posY_cm
			#mambo.smart_sleep(0.1)
		else:
			print("camera missed")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			break
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	correct_yaw(math.copysign(90,ang),tol_yaw)
	print("Controle fino em Y...")
	while(abs(y_d-posY_cm)>tol):
			pitch = math.copysign(vel_y_fino,ang*(y_d-posY_cm))
			mambo.fly_direct(roll=roll_base, pitch=pitch+pitch_base, yaw=0, vertical_movement=0, duration=0.2)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	print("done!\n")
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)

def controleDiag(x_d,y_d,ang_d,tol):
	global roll_base, pitch_base
	print("Controle diagonal...")
	posx = posX_cm
	posy = posY_cm
	erro_x = x_d - posX_cm
	erro_y = y_d - posY_cm

	while(posX_cm == 0):
		posx = posX_cm
		posy = posY_cm
		erro_x = x_d - posX_cm
		erro_y = y_d - posY_cm

	pitch = 0
	roll = 0
	count = 0
	vel_y = 17
	vel_y_fino = 40
	# range_bias = 7
	while ((abs(erro_y)>tol) or (abs(erro_x)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			ang_d = math.atan2(erro_y,-erro_x)*180/math.pi
			#roll_b = bias_roll3(posx, posy, ang_d)
			roll_b = bias_roll()
			if(posZ_cm>65):
				altitude(50,12)
			if(abs(ang_d - ang)>tol_ang):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				correct_yaw(ang_d,tol_yaw)
			if(abs(mambo.sensors.speed_x) > 0.25):
				# pitch = 0
				print("slowing...")
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				mambo.flat_trim()
			elif(abs(erro_y) < 20):
				if(abs(mambo.sensors.speed_x) > 0.13):
					# pitch = 0
					print("slowing close...")
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
					mambo.flat_trim()
				else:
					#pitch = vel_y
					pitch = math.copysign(vel_y,erro_y*ang)
			else:
				#pitch = vel_y
				pitch = math.copysign(vel_y,erro_y*ang)
			mambo.fly_direct(roll=roll_b, pitch=pitch+pitch_base, yaw=0, vertical_movement=0, duration=0.1)
			erro_x = x_d - posX_cm
			erro_y = y_d - posY_cm
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x, " erro_y: %.2f" % erro_y, \
			" pitch: ", pitch, " roll: %.2f" % roll_b, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
			#mambo.smart_sleep(0.1)
		else:
			print("camera missed")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			mambo.safe_land(5)
			mambo.smart_sleep(1)
			print("disconnect")
			mambo.disconnect()
			break
	#mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	#correct_yaw(ang_d,tol_yaw)
	
	#print("Controle fino diagonal...")
	#while(abs(y_d-posY_cm)>tol):
	#		pitch = math.copysign(vel_y_fino,ang*(y_d-posY_cm))
	#		mambo.fly_direct(roll=5, pitch=pitch, yaw=0, vertical_movement=0, duration=0.3)
	#		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	#		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	
	print("done!\n")
	print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)

def altitude(h,vel):
	count = 0
	tol=2.5
	#tol = 7
	print("going to altitude %.1f centimeters..." % h)
	while(abs(posZ_cm-h)>tol):
		while ((posZ_cm>h+tol or posX_cm == 0) or count < 4):
			if(posZ_cm<=h+tol):
				count = count + 1
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
				print("ok")
			else:
				count = 0
				if(posX_cm == 0):
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
				else:
					if(posZ_cm<h+10):
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-vel/2, duration=0.1)
					else:
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-vel, duration=0.1)
			print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
		
		while ((posZ_cm<h-tol or posX_cm == 0) or count < 4):
			if(posZ_cm>=h-tol):
				count = count + 1
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
				print("ok")
			else:
				count = 0
				if(posX_cm == 0):
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
				else:
					if(posZ_cm>h+10):
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=vel/2, duration=0.1)
					else:
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=vel, duration=0.1)
			print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
			
	print("done!\n")
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)


posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
pitch_base = 0
roll_base = 0
obj_x = 0; obj_y = 0; obj_z = 0
size1 = 0; size2 = 0
wp = []
grid_scale = 10 # espaço entre nós = 10 cm

drone_node()

mamboAddr = "D0:3A:5E:20:E6:21"
mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)
success = 1;
if(success):
	try:
		mambo.ask_for_state_update()
		mambo.smart_sleep(0.5)
		print("battery: ", mambo.sensors.battery)
		
		print("taking off!")
		mambo.safe_takeoff(5)
		mambo.smart_sleep(1)
		max_tilt = 30
		mambo.set_max_tilt(max_tilt)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm , " ang: %.1f\n" % ang)
		
		sleep(2)
		print(wp)
		
		tol = 5.5 #tolerancia de waypoint (quadrada no momento para controle45, nao circular)
		tol_yaw = 4.6 #tol_yaw é tolerancia para correçao do angulo
		tol_ang = 10 #tol_ang é a tolerancia de erro de ang para acontecer correct_yaw
		
		# altitude(50,12)
		size1=1
		size2=4
		wp = numpy.zeros((size1,size2,2))
		wp[0,0,0] = 0
		wp[0,0,1] = 0
		wp[0,1,0] = posX_cm-45
		wp[0,1,1] = posY_cm+45
		wp[0,2,0] = posX_cm
		wp[0,2,1] = posY_cm+90
		# wp[0,3,0] = posX_cm
		# wp[0,3,1] = posY_cm
		for i in range(size1): #numero de trajetorias
			for j in range(size2): #waypoints em cada trajetoria
				if(i==0 and j==0):
					print("skip first wp") #primeiro wp é o local inicial
				else:
					#define o waypoint
					x = wp[i,j,0]
					y = wp[i,j,1]
					#x = posX_cm-40
					#y = posY_cm+40
					if(not(x==0 and y==0)):
						count = 0;
						print("Indo para waypoint (",x,",",y,")...")
						dx = x - posX_cm
						dy = y - posY_cm
						while(posX_cm==0):
							dx = x - posX_cm
							dy = y - posY_cm
							count = count + 1					
							print("camera missed")
							mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
							if(count>13):
								print("count exceded! landing...")
								print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
								count = 0
								mambo.safe_land(5)
								mambo.smart_sleep(1)
								print("disconnect")
								mambo.disconnect()
								break						
						print("dx: ",dx,", dy: ",dy)
						#definição do angulo de partida
						ang_partida = math.atan2(dy,-dx)*180/math.pi #usando -dx para corrigir diferença entre angulo calculado e angulo real/camera
						ang_partida_round = round(ang_partida/45)*45
						#rotaçao para angulo desejado
						print("angulo: ",ang_partida)
						correct_yaw(ang_partida, tol_yaw)
						#movimentar drone na direçao desejada até wp
						if(abs(ang_partida_round)==0 or abs(ang_partida_round)==180):
							controleX(x, tol)
						elif(abs(ang_partida_round)==90):
							controleY(y, tol)
						else:
							controleDiag(x,y,ang_partida,tol)
						
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
						print("\nWaypoint (",x,",",y,") alcançado!\n")
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
						mambo.smart_sleep(2)
						altitude(50,12)
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		
		# altitude(50,12)
		print("done all!")
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
		
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-20, duration=2)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		
	except:
		print("abortado: ", sys.exc_info()[0])
		
	print("landing")
	mambo.safe_land(5)
	
	print("battery: ", mambo.sensors.battery)
	print("disconnect")
	mambo.disconnect()
	
	#EOF
else:
	print("connection problem, exiting")
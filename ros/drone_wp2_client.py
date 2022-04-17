#!/usr/bin/env python3

from __future__ import print_function
from time import sleep
from threading import Thread
import math
from pyparrot.Minidrone import Mambo
import sys
import os
import numpy

#udp
import struct
import socket

#ros
# import rospy
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import UInt32MultiArray

class server(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.killed = False
		
	def kill(self): 
		self.killed = True
		
	def run(self):
		try:
			global timestamp, posX_cm, posY_cm, posZ_cm, ang, ang_d, vel_record, timestamp_old
			# ip = '127.0.0.1'
			ip = '192.168.0.41'
			port = 12000
			buff_size = 1024
			addr = (ip, port)
			server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			server_socket.bind(('', port))
			while True:
				data, server = server_socket.recvfrom(buff_size)
				data = struct.unpack('<9f', data)
				[_,_, _, _, _, aux, _, _, _] = data
				if(aux>0):
					[timestamp, _, _, _, _, posX_cm, posY_cm, posZ_cm, ang] = data #drone 2 pose
				if(self.killed):
					break
					
				if(savefile and (timestamp_old!=timestamp)):
					X.append(posX_cm)
					Y.append(posY_cm)
					Z.append(posZ_cm)
					A.append(ang)
					if(vel_record):
						VX.append(mambo.sensors.speed_x)
						VY.append(mambo.sensors.speed_y)
						VZ.append(mambo.sensors.speed_z)
					else:
						VX.append(0)
						VY.append(0)
						VZ.append(0)
					if(ang_d!=-999):
						E.append(abs(ang_d-ang))
					else:
						E.append(0)
					T.append(timestamp)
					timestamp_old = timestamp
		
		except KeyboardInterrupt:
			print('Interrupt by user...')
			self.killed = True

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
	#ang_d = -ang_d #angulo que sera usado para calculos eh diferente do angulo calculado pela camera
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
	vel_base = 15
	vel = vel_base
	slow_vel_ang = 18
	high_vel_ang = 36
	print("corrigindo yaw...")
	while(ang==0 and posX_cm > 900):
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
		if(ang == 0 and posX_cm > 900):
			count = count + 1
			if(count > 12):
				print("count exceeded: ang = 0")
				mambo.safe_land(5)
				mambo.smart_sleep(1)
				print("disconnect")
				mambo.disconnect()
				return
		else:
			if(abs(ang-base)>high_vel_ang):
				vel = vel_base*1.5
			elif(abs(ang-base)<slow_vel_ang):
				vel = vel_base/2
			else:
				vel = vel_base
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

def correct_yaw2(base,tol):
	global ang, posX_cm
	count = 0
	vel_base = 15
	vel = vel_base
	slow_vel_ang = 18
	high_vel_ang = 36
	print("corrigindo yaw...")
	while(ang==0 and posX_cm > 900):
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
	
	if(abs(ang-base)>high_vel_ang):
		vel = vel_base*1.5
	elif(abs(ang-base)<slow_vel_ang):
		vel = vel_base/2
	else:
		vel = vel_base
	if(ang-base>0):
		if(ang-base>180):
			yaw=vel
		else:
			yaw=-vel
	if(ang-base<0):
		if(ang-base<-180):
			yaw=-vel
		else:
			yaw=vel
	yaw = yaw * 1.5
	return yaw

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
			
			#bias_roll2 implementado na propria funcao controleY
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

def controleDiag(x_d,y_d,tol):
	global roll_base, pitch_base, ang_d
	print("Controle diagonal...")
	posx = posX_cm
	posy = posY_cm
	erro_x = x_d - posX_cm
	erro_y = y_d - posY_cm

	while(posX_cm > 900):
		posx = posX_cm
		posy = posY_cm
		erro_x = x_d - posX_cm
		erro_y = y_d - posY_cm

	pitch = 0
	roll = 0
	roll_b = 0
	count = 0
	vel_y = 10
	vel_y_fino = 40
	vel_max = 0.33
	vel_max_close = 0.18
	dist = math.sqrt(erro_y**2 + erro_x**2)
	# range_bias = 7
	#(abs(erro_y)>tol) or (abs(erro_x)>tol)
	while (dist>tol or (posX_cm > 900)):
		if(posX_cm < 900):
			count = 0
			ang_d = math.atan2(erro_y,-erro_x)*180/math.pi
			#roll_b = bias_roll3(posx, posy, ang_d)
			# roll_b = bias_roll()
			roll_b = 0
			
			if(posZ_cm>65):
				altitude(50,12)
				
			# if(abs(ang_d - ang)>tol_ang):
				# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				# correct_yaw(ang_d,tol_yaw)
				
			if(abs(mambo.sensors.speed_x) > vel_max):
				# pitch = 0
				print("slowing...")
				# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				# mambo.flat_trim()
				pitch = vel_y/2
			elif(abs(erro_y) < 20):
				if(abs(mambo.sensors.speed_x) > vel_max_close):
					# pitch = 0
					print("slowing close...")
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
					# mambo.flat_trim()
					pitch = vel_y/2
				else:
					pitch = vel_y
					# pitch = math.copysign(vel_y,erro_y*ang)
			else:
				pitch = vel_y
				# pitch = math.copysign(vel_y,erro_y*ang)
				
			if(abs(ang_d - ang)>tol_ang):
				# if(dist>tol*3):
					# roll_b = roll_b + correct_yaw2(ang_d,tol_yaw)
					# pitch = pitch/2
				# else:
				# print("correct yaw dentro do raio2")
				roll_b = 0
				correct_yaw(ang_d,tol_yaw)
			else:
				roll_b = 1*(ang_d - ang)
				
			mambo.fly_direct(roll=roll_base+roll_b, pitch=pitch+pitch_base, yaw=0, vertical_movement=0, duration=0.1)
			erro_x = x_d - posX_cm
			erro_y = y_d - posY_cm
			dist = math.sqrt(erro_y**2 + erro_x**2)
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
	global loaded
	count = 0
	tol=2.5
	vel = vel*(1+0.67*loaded)
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


def callback(data):
	global timestamp, posX_cm, posY_cm, posZ_cm, ang, obj_x, obj_y, obj_z
	posX_cm = data.data[0]; posY_cm = data.data[1]; posZ_cm = data.data[2]; ang = data.data[3]
	obj_x = data.data[8]; obj_y = data.data[9]; obj_z = data.data[10]
	if(savefile):
		X.append(posX_cm)
		Y.append(posY_cm)
		Z.append(posZ_cm)
		A.append(ang)
		T.append(timestamp)
	
def callback_wp(data):
	global wp, size1, size2
	size1 = data.layout.dim[0].size
	size2 = data.layout.dim[1].size
	stride2 = data.layout.dim[1].stride
	wp = numpy.zeros((size1,size2,2))
	for i in range(size1):
		for j in range(size2):
			wp[i,j,0] = data.data[stride2*i + 2*j]*grid_scale
			wp[i,j,1] = data.data[stride2*i + 2*j + 1]*grid_scale
				
def drone_node():
	rospy.init_node('drone_node', anonymous=True, disable_signals=True)
	rospy.Subscriber("camera_topic", Float32MultiArray, callback)
	rospy.Subscriber("wp", UInt32MultiArray, callback_wp)

timestamp = 0; timestamp_old = 0;
posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
ang_d = -999
pitch_base = 0
roll_base = 5
obj_x = 0; obj_y = 0; obj_z = 0
size1 = 0; size2 = 0
wp = []
grid_scale = 10 # espaco entre nos = 10 cm
#mamboAddr = "D0:3A:5E:20:E6:21"
mamboAddr = "D0:3A:BD:55:E6:23" #segundo drone

savefile = 0
vel_record = 0
# SAVE DATA IN TXT
if(savefile):
	X=[];Y=[];Z=[];A=[];VX=[];VY=[];VZ=[];E=[];T=[]
	i=1
	name = "data_saved%d.txt"%i
	while(os.path.isfile(name)):
		i=i+1
		name = "data_saved%d.txt"%i
	f = open(name, 'w')
	print("opening %s..."%name)

# drone_node()
thread1 = server()
thread1.start()

mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)
success = 1;
if(success):
	try:
	
		# while(1):
			# print("timestamp: %.3f" % timestamp, " posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm , " ang: %.1f\n" % ang)
			# sleep(0.05)
		mambo.ask_for_state_update()
		mambo.smart_sleep(0.5)
		print("battery: ", mambo.sensors.battery)
		
		vel_record = 1
		
		print("taking off!")
		# mambo.safe_takeoff(5)
		mambo.smart_sleep(1)
		max_tilt = 30
		mambo.set_max_tilt(max_tilt)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm , " ang: %.1f\n" % ang)
		
		tol = 7 #tolerancia de waypoint (quadrada no momento para controle45, nao circular)
		tol_yaw = 5 #tol_yaw eh tolerancia para correcao do angulo
		tol_ang = 15 #tol_ang eh a tolerancia de erro de ang para acontecer correct_yaw
		loaded = 0
		
		# altitude(50,12)
		size1=1
		size2=4
		# y = 87
		# x = 109
		wp = numpy.zeros((size1,size2,2))
		wp[0,0,0] = 0
		wp[0,0,1] = 0
		# wp[0,1,0] = 109 #posX_cm
		# wp[0,1,1] = 137 #posY_cm+50
		wp[0,1,0] = posX_cm
		wp[0,1,1] = posY_cm-50
		# wp[0,2,0] = 55 #posX_cm-40
		# wp[0,2,1] = 87 #posY_cm+10
		# wp[0,3,0] = 109 #posX_cm-40
		# wp[0,3,1] = 85 #posY_cm+10
		# wp[0,4,0] = posX_cm
		# wp[0,4,1] = posY_cm
		for i in range(size1): #numero de trajetorias
			for j in range(size2): #waypoints em cada trajetoria
				if(i==0 and j==0):
					print("skip first wp") #primeiro wp eh o local inicial
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
						while(posX_cm>900):
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
						#definicao do angulo de partida
						ang_d = math.atan2(dy,-dx)*180/math.pi #usando -dx para corrigir diferenca entre angulo calculado e angulo real/camera
						ang_partida = ang_d
						ang_d_round = round(ang_d/45)*45
						#rotacao para angulo desejado
						print("angulo: ",ang_d)
						correct_yaw(ang_d, tol_yaw)
						#movimentar drone na direcao desejada ate wp
						# if(abs(ang_partida_round)==0 or abs(ang_partida_round)==180):
							# controleX(x, tol)
						# elif(abs(ang_partida_round)==90):
							# controleY(y, tol)
						# else:
						
						altitude(50,12)
						
						controleDiag(x,y,tol)
						
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
						print("\nWaypoint (",x,",",y,") alcancado!\n")
						
						if(j==1):
							print("\ncorrect yaw to grab object...\n")
							correct_yaw(ang_partida, tol_yaw)
							# while(posZ_cm > 35+5.5):
								# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-5, duration=0.1)
							print("\n5s to grab object...\n")
							mambo.smart_sleep(5)
							mambo.close_claw()
							mambo.smart_sleep(1)
							loaded = 1
							altitude(50,12)
							tol = 5
						if(j==2):		
							correct_yaw(-90, tol_yaw)
							print("\nlet it go object...\n")						
							mambo.open_claw()
							mambo.smart_sleep(1)
							loaded = 0
						
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=2)
						
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		
		print("done all!")
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=2)
		
		# while(posZ_cm > 35):
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-5, duration=0.1)
		# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
		# mambo.close_claw()
		# mambo.smart_sleep(2)
		
		# while(posZ_cm > 30):
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-20, duration=None)
			
		# altitude(30,20)
		
		#print("going down to land in 3 seconds...")
		#mambo.smart_sleep(3)
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-20, duration=1.5)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		
	except:
		print("abortado: ", sys.exc_info()[0])
		
	print("landing")
	mambo.safe_land(5)
	
	thread1.kill()
	sleep(0.3)
	
	print("battery: ", mambo.sensors.battery)
	print("disconnect")
	mambo.disconnect()
	if(savefile):
		print("saving...")
		for item in range(len(Z)):		
			f.write("%.1f," % X[item])
			f.write("%.1f," % Y[item])
			f.write("%.1f," % Z[item])
			f.write("%.1f," % A[item])
			f.write("%.3f," % VX[item])
			f.write("%.3f," % VY[item])
			f.write("%.3f," % VZ[item])
			f.write("%.1f," % E[item])
			f.write("%.3f\r\n" % T[item])
		print("saved %s!"%name)
		f.close()
	#EOF
else:
	print("connection problem, exiting")
	if(savefile):
		print("saving...")
		for item in range(len(Z)):		
			f.write("%.1f," % X[item])
			f.write("%.1f," % Y[item])
			f.write("%.1f," % Z[item])
			f.write("%.1f," % A[item])
			f.write("%.3f," % VX[item])
			f.write("%.3f," % VY[item])
			f.write("%.3f," % VZ[item])
			f.write("%.1f," % E[item])
			f.write("%.3f\r\n" % T[item])
		print("saved %s!"%name)
		f.close()
	thread1.kill()
	
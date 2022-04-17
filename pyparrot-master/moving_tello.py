from __future__ import print_function
from easytello import tello
from threading import Thread
from time import sleep
import math
import sys
import numpy
import threading
import ctypes

import socket
import struct

class rc_tello(Thread):
	def __init__(self):
		Thread.__init__(self)
		
	def run(self):
		try:
			global roll,pitch,vertical,yaw, movenow
			while True:
				if(movenow==1):
					drone.rc_control(roll, pitch, vertical, yaw)
				if(movenow==-1):
					drone.rc_control(0, 0, 0, 0)
				sleep(0.1)
		except KeyboardInterrupt:
			print('Interrupt by user...')
			drone.land()
			
	def get_id(self): 
		# returns id of the respective thread 
		if hasattr(self, '_thread_id'): 
			return self._thread_id 
		for id, thread in threading._active.items(): 
			if thread is self: 
				return id	
	
	def raise_exception(self): 
		thread_id = self.get_id() 
		res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 
			  ctypes.py_object(SystemExit)) 
		if res > 1: 
			ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0) 
			print('Exception raise failure') 

def restart():
	global movenow, thread1
	thread1.raise_exception()
	sleep(0.1)
	movenow=0
	thread1 = rc_tello()
	thread1.start()

def finish():
	global thread1
	thread1.raise_exception()

def correct_yaw(base,tol):
	global ang, posX_cm, movenow, pitch, roll, yaw, vertical
	count = 0
	vel_yaw = 10
	vel = vel_yaw
	print("corrigindo yaw to %.1f ..." % base)
	while(ang==0 and posX_cm == 0):
		count = count + 1
		movenow=-1
		sleep(0.2)
		restart()
		print("camera miss...")
		if(count>12):			
			print("count exceeded: ang = 0")
			drone.land()
			return
	count = 0
	
	while (abs(ang-base)>tol) or (ang == 0):
		if(ang == 0 and posX_cm == 0):
			count = count + 1
			if(count > 12):
				print("count exceeded: ang = 0")
				drone.land()
				return
		else:
			if(ang-base>0):
				if(ang-base>180):
					drone.cw(4)
				else:
					drone.ccw(4)
			if(ang-base<0):
				if(ang-base<-180):
					drone.ccw(4)
				else:
					drone.cw(4)
			count = 0
		print("yaw: %.1f" % ang)
	movenow=-1
	sleep(0.5)
	restart()
	print("done adjusting yaw!")
	print("new yaw: %.1f\n" % ang)
	return

def controleX(x_d,tol):
	global pitch, roll, vertical, yaw, movenow
	print("Controle no eixo X...")
	posy = posY_cm
	posx = posX_cm
	erro_x = x_d - posX_cm
	
	while(posX_cm == 0):
		posy = posY_cm
		erro_x = x_d - posX_cm
	
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
			if(not(abs(ang)<tol_ang or abs(ang)>(180-tol_ang))): 
				movenow=-1
				sleep(0.4)
				restart()
				correct_yaw((ang_d),tol_yaw) 
			
			pitch = math.copysign(1,erro_x/dir)*vel
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x)
			movenow=1
			sleep(0.2)
			restart()
			erro_x = x_d - posX_cm
		else:
			print("camera missed")
			movenow=-1
			sleep(0.15)
			restart()
			sleep(0.2)
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			drone.land()
			break
	movenow=-1
	sleep(0.5)
	restart()

def controleY(y_d,tol):
	global pitch, roll, vertical, yaw, movenow
	print("Controle no eixo Y...")
	posx = posX_cm
	erro_y = y_d - posY_cm
	while(posX_cm == 0):
		posx = posX_cm
		erro_y = y_d - posY_cm

	count = 0
	vel_y = 10
	vel_y_fino = 40
	# range_bias = 7
	
	if(erro_y<0):
		ang_d = -90
	else:
		ang_d = 90
	
	while ((abs(erro_y)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0		
			if(abs(ang)<(90-tol_ang) or abs(ang)>(90+tol_ang)):
				movenow=-1
				sleep(0.4)
				restart()
				correct_yaw(math.copysign(90,ang),tol_yaw)

			pitch = math.copysign(vel_y,erro_y*ang)
			movenow=1
			sleep(0.2)
			restart()
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_y: %.2f" % erro_y)
			erro_y = y_d - posY_cm
		else:
			print("camera missed")
			movenow=-1
			sleep(0.3)
			restart()
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			drone.land()
			break
	print("done!\n")

	movenow=-1
	sleep(0.5)
	restart()

def controleDiag(x_d,y_d,ang_d,tol):
	global pitch, roll, vertical, yaw, href, movenow
	print("Controle diagonal...")
	posx = posX_cm
	posy = posY_cm
	erro_x = x_d - posX_cm
	erro_y = y_d - posY_cm
	# erro_y = -erro_y

	while(posX_cm == 0):
		posx = posX_cm
		posy = posY_cm
		erro_x = x_d - posX_cm
		erro_y = y_d - posY_cm
		# erro_y = -erro_y

	count = 0
	vel_y = 10
	vel_y_fino = 40
	while ((abs(erro_y)>tol) or (abs(erro_x)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			ang_d = math.atan2(erro_y,erro_x)*180/math.pi
			# roll_b = bias_roll()
			# if(posZ_cm>href+13):
				# altitude(href,10)
			if(abs(ang_d - ang)>tol_ang):
				movenow=-1
				sleep(0.4)
				restart()
				correct_yaw(ang_d,tol_yaw)

			pitch = math.copysign(vel_y,erro_y*ang)
			movenow=1
			sleep(0.2)
			restart()
			erro_x = x_d - posX_cm
			erro_y = y_d - posY_cm
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x, " erro_y: %.2f" % erro_y)
		else:
			print("camera missed")
			movenow=-1
			sleep(0.3)
			restart()
			count = count + 1
		if(count>13):
			print("count exceded! landing...")
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
			count = 0
			drone.land()
			break
	
	print("done!\n")
	print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	movenow=-1
	sleep(0.5)
	restart()

def altitude(h,vel):
	global pitch, roll, vertical, yaw, movenow
	count = 0
	count_cam = 0
	tol=5
	#tol = 7
	print("going to altitude %.1f centimeters..." % h)
	while(abs(posZ_cm-h)>tol):
		while ((posZ_cm>h+tol or posX_cm == 0) or count < 4):
			if(posZ_cm<=h+tol):
				count = count + 1
				movenow=-1
				sleep(0.3)
				restart()
				print("ok")
			else:
				count = 0
				if(posX_cm == 0):
					movenow=-1
					sleep(0.3)
					restart()
					count_cam = count_cam + 1
					print("camera missed")
					if(count_cam > 13):
						print("count exceded! landing...")
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
						count = 0
						drone.land()
						break			
				else:
					count_cam = 0
					if(posZ_cm<h+10):
						vertical = -6
						movenow=1
						sleep(0.3)
						restart()
					else:
						vertical = -9
						movenow=1
						sleep(0.3)
						restart()
			print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
		
		while ((posZ_cm<h-tol or posX_cm == 0) or count < 4):
			if(posZ_cm>=h-tol):
				count = count + 1
				movenow=-1
				sleep(0.3)
				restart()
				print("ok")
			else:
				count = 0
				if(posX_cm == 0):
					movenow=-1
					sleep(0.3)
					restart()
					count_cam = count_cam + 1
					print("camera missed")
					if(count_cam > 13):
						print("count exceded! landing...")
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
						count = 0
						drone.land()
						break	
				else:
					if(posZ_cm>h+10):
						vertical = -10
						movenow=1
						sleep(0.2)
						restart()
					else:
						vertical = -5
						movenow=1
						sleep(0.2)
						restart()
			print("X=%.1f" % posX_cm,"cm    Y=%.1f" % posY_cm, "cm    Z=%.1f" % posZ_cm, "cm")
			
	print("done!\n")
	movenow=-1
	sleep(0.5)
	restart()
		
class server(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.killed = False
		
	def kill(self): 
		self.killed = True
		
	def run(self):
		try:
			global posX_cm, posY_cm, posZ_cm, ang
			ip = '127.0.0.1'
			port = 12000
			buff_size = 1024
			addr = (ip, port)
			server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			server_socket.bind(('', port))
			while True:
				data, server = server_socket.recvfrom(buff_size)
				data = struct.unpack('<4f', data)
				[posX_cm, posY_cm, posZ_cm, ang] = data
				if(self.killed):
					break
		except KeyboardInterrupt:
			print('Interrupt by user...')
			self.killed = True

posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
pitch=0; roll=0; vertical=0; yaw=0
obj_x = 0; obj_y = 0; obj_z = 0
size1 = 0; size2 = 0
wp = []
grid_scale = 10 # espaço entre nós = 10 cm

drone = tello.Tello()

movenow=0
thread2 = server()
thread2.start()
thread1 = rc_tello()
thread1.start()

try:
	print(drone.get_battery())
	drone.takeoff()
	print("taking off!")

	print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm , " ang: %.1f\n" % ang)

	tol = 5.5 #tolerancia de waypoint (quadrada no momento para controle45, nao circular)
	tol_yaw = 5 #tol_yaw é tolerancia para correçao do angulo
	tol_ang = 11 #tol_ang é a tolerancia de erro de ang para acontecer correct_yaw
	href = 50
	
	# altitude(50,12)
	size1=1
	size2=5
	wp = numpy.zeros((size1,size2,2))
	x = posX_cm
	y = posY_cm
	wp[0,0,0] = x
	wp[0,0,1] = y
	wp[0,1,0] = x-45
	wp[0,1,1] = y+45
	wp[0,2,0] = x
	wp[0,2,1] = y+90
	wp[0,3,0] = x-45
	wp[0,3,1] = y+45
	wp[0,4,0] = x
	wp[0,4,1] = y
	
	altitude(href,10)
	for i in range(size1): #numero de trajetorias
		for j in range(size2): #waypoints em cada trajetoria
			if(i==0 and j==0):
				print("skip first wp") #primeiro wp é o local inicial
			else:
				#define o waypoint
				x = wp[i,j,0]
				y = wp[i,j,1]
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
						movenow=-1
						sleep(0.2)
						restart()
						if(count>13):
							print("count exceded! landing...")
							print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
							count = 0
							drone.land()
							break						
					print("dx: ",dx,", dy: ",dy)
					#definição do angulo de partida
					ang_partida = math.atan2(dy,-dx)*180/math.pi #usando -dx para corrigir diferença entre angulo calculado e angulo real/camera
					ang_partida_round = round(ang_partida/45)*45
					#rotaçao para angulo desejado
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
					movenow=-1
					sleep(1)
					restart()
					altitude(href,10)
					print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	print("done all!")
	movenow=-1
	sleep(5)
	restart()
	print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	
except:
	print("abortado: ", sys.exc_info()[0])
	
print("landing")
thread2.kill()
finish()
drone.land()
sleep(1)
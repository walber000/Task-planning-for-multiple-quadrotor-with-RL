from __future__ import print_function
from pyparrot.Minidrone import Mambo
from threading import Thread
from time import sleep
import math
import sys
import numpy

import socket
import struct

def bias_roll():
	global roll_base
	roll_b = roll_base
	if(mambo.sensors.speed_y<-0.015):
		roll_b = 5 + roll_base
	if(mambo.sensors.speed_y>0.015):
		roll_b = -5 + roll_base
	if(mambo.sensors.speed_y<-0.03):
		roll_b = 10 + roll_base
	if(mambo.sensors.speed_y>0.03):
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
	return roll_b

def correct_yaw(base,tol):
	global ang, posX_cm
	count = 0
	vel_yaw = 14
	vel = vel_yaw
	print("corrigindo yaw to %.1f ..." % base)
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
			if(abs(ang-base)<15):
				vel = vel_yaw/2 #reduzir velocidade se erro de angulo < 15 graus
			else:
				vel = vel_yaw
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
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.8)
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
	# erro_y = -erro_y
	while(posX_cm == 0):
		posx = posX_cm
		erro_y = y_d - posY_cm
		# erro_y = -erro_y

	pitch = 0
	roll = 0
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
			roll_b = bias_roll()		
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
			# erro_y = -erro_y
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
	global roll_base, pitch_base, href
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

	pitch = 0
	roll = 0
	count = 0
	vel_y = 10
	vel_y_fino = 40
	while ((abs(erro_y)>tol) or (abs(erro_x)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			ang_d = math.atan2(erro_y,erro_x)*180/math.pi
			roll_b = bias_roll()
			# if(posZ_cm>href+13):
				# altitude(href,10)
			if(abs(ang_d - ang)>tol_ang):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				correct_yaw(ang_d,tol_yaw)
			if(abs(mambo.sensors.speed_x) > 0.25):
				print("slowing...")
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				mambo.flat_trim()
			elif(abs(erro_y) < 20):
				if(abs(mambo.sensors.speed_x) > 0.13):
					print("slowing close...")
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
					mambo.flat_trim()
				else:
					pitch = math.copysign(vel_y,erro_y*ang)
			else:
				pitch = math.copysign(vel_y,erro_y*ang)
			mambo.fly_direct(roll=roll_b, pitch=pitch+pitch_base, yaw=0, vertical_movement=0, duration=0.1)
			erro_x = x_d - posX_cm
			erro_y = y_d - posY_cm
			# erro_y = -erro_y
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x, " erro_y: %.2f" % erro_y, \
			" pitch: ", pitch, " roll: %.2f" % roll_b, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
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
	
	print("done!\n")
	print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)

def altitude(h,vel):
	count = 0
	tol=5
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
		
class server(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.killed = False
		
	def kill(self): 
		self.killed = True
		
	def run(self):
		try:
			global posX_cm, posY_cm, posZ_cm, ang
			ip = '192.168.1.129'
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

mamboAddr = "D0:3A:5E:20:E6:21"
#mamboAddr = "D0:3A:BD:55:E6:23" #segundo drone

posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
pitch_base = 0
roll_base = 0
obj_x = 0; obj_y = 0; obj_z = 0
size1 = 0; size2 = 0
wp = []
grid_scale = 10 # espaço entre nós = 10 cm

mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)
# success = 1;

thread1 = server()
thread1.start()

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
		# print(wp)
	
		tol = 6 #tolerancia de waypoint (quadrada no momento para controle45, nao circular)
		tol_yaw = 5 #tol_yaw é tolerancia para correçao do angulo
		tol_ang = 11 #tol_ang é a tolerancia de erro de ang para acontecer correct_yaw
		href = 65
		
		# altitude(50,12)
		size1=1
		size2=5
		wp = numpy.zeros((size1,size2,2))
		x = posX_cm
		y = posY_cm
		wp[0,0,0] = x
		wp[0,0,1] = y
		wp[0,1,0] = x+45
		wp[0,1,1] = y-45
		wp[0,2,0] = x+90
		wp[0,2,1] = y
		wp[0,3,0] = x+45
		wp[0,3,1] = y-45
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
						ang_partida = math.atan2(dy,dx)*180/math.pi #usando -dx para corrigir diferença entre angulo calculado e angulo real/camera
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
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
						mambo.smart_sleep(2)
						altitude(href,10)
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
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
	thread1.kill() 
else:
	print("connection problem, exiting")
	thread1.kill() 

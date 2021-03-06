#!/usr/bin/env python3

from __future__ import print_function
from time import sleep
from threading import Thread
import math
from pyparrot.Minidrone import Mambo
import sys

import numpy
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt32MultiArray

def bias_roll(posx, dir): #dir=1 -> X, Y+ / dir=-1 -> X, Y- / dir=2 -> Y, X+ / dir=-2 -> Y, X-
	global posX_cm, posY_cm
	base = -10
	
	if(mambo.sensors.speed_y<-0.01):
		base = base + 5
	if(mambo.sensors.speed_y>0.01):
		base = base - 5
	
	roll_b = base
	
	if dir == 1 or dir == -1:
		pos = posX_cm
	elif dir == 2 or dir == -2:
		pos = posY_cm
	
	if(abs(dir)==2):
		dir = -dir/2
	
	if(pos<(-5+posx)):
		roll_b = base + 15*dir
	if(pos>(5+posx)):
		roll_b = base - 15*dir
		
	if(pos<(-15+posx)):
		roll_b = base + 30*dir
	if(pos>(15+posx)):
		roll_b = base - 30*dir

	return roll_b

def bias_roll2(posx, dir): #dir=1 -> X, Y+ / dir=-1 -> X, Y- / dir=2 -> Y, X+ / dir=-2 -> Y, X-
	roll_b = 5
	global posX_cm, posY_cm
		
	if dir == 1 or dir == -1:
		pos = posX_cm
	elif dir == 2 or dir == -2:
		pos = posY_cm
	
	if(abs(dir)==2):
		dir = -dir/2
	
	if(pos<(-5+posx)):
		print("bias: ", (30*dir))
		mambo.fly_direct(roll=30*dir, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
		mambo.flat_trim()
	if(pos>(5+posx)):
		print("bias: ", (-30*dir))
		mambo.fly_direct(roll=-30*dir, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
		mambo.flat_trim()
	
	return roll_b
	
def bias_roll3(posx, posy, ang_d): #posx e posy sao posicao inicial do waypoint de onde o drone saiu, ang eh o angulo desejado para aquela trajetoria saindo do wp, nao usar 90graus
	roll_b = 5

	global posX_cm, posY_cm
	tol = 5 #drone deve se manter dentro de um canal de tol cm para cada lado, enquanto segue a linha central (mov desejado) de angulo ang
	#ang_d = -ang_d #angulo que será usado para calculos é diferente do angulo calculado pela camera
	ang_rad = -ang_d*math.pi/180

	dx = posX_cm - posx
	dy = posY_cm - posy
	
	dy_esperado = dx * math.tan(ang_rad)
	dy_max = tol*math.sqrt((1+math.tan(ang_rad)**2))

	print("dy: ",dy,", range bias: ",(dy_esperado-dy_max)," - ",(dy_esperado+dy_max))

	if(abs(ang_d)<90):
		sign = 1
	else:
		sign = -1

	while((dy<(dy_esperado-dy_max)) or (dy>(dy_esperado+dy_max))):
		if(abs(ang_d - ang)>tol_ang) :
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
			correct_yaw(ang_d,tol_yaw) 
		if(dy<(dy_esperado-dy_max)):
			print("bias: ", (math.copysign(30,sign)))
			mambo.fly_direct(roll=math.copysign(30,sign), pitch=0, yaw=0, vertical_movement=0, duration=0.7)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			mambo.flat_trim()
		if(dy>(dy_esperado+dy_max)):
			print("bias: ", (math.copysign(30,-sign)))
			mambo.fly_direct(roll=math.copysign(30,-sign), pitch=0, yaw=0, vertical_movement=0, duration=0.7)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			mambo.flat_trim()
		dx = posX_cm - posx
		dy = posY_cm - posy
		dy_esperado = dx * math.tan(ang_rad)
		dy_max = tol*math.sqrt((1+math.tan(ang_rad)**2))
		print("dy: ",dy,", range bias: ",(dy_esperado-dy_max)," - ",(dy_esperado+dy_max))
	
	return roll_b

def correct_yaw(base,tol):
	global ang
	count = 0
	vel = 10
	print("corrigindo yaw...")
	while(ang==0):
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
		if(ang == 0):
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
	print("Controle no eixo X...")
	posy = posY_cm
	erro_x = x_d - posX_cm
	
	while(posX_cm == 0):
		posy = posY_cm
		erro_x = x_d - posX_cm
		
	pitch = 0
	count = 0
	vel = 13
	vel_fino = 40

	if(erro_x<0):
		dir = -2
	else:
		dir = 2
	while (((abs(erro_x)>tol) and (erro_x/dir > 0)) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			roll = bias_roll2(posy, dir)
			
			if(not(abs(ang)<tol_ang or abs(ang)>(180-tol_ang))): 
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				correct_yaw((45*dir + 90),tol_yaw) 
			
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
			" pitch: ", pitch, " roll: ", roll, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.1)
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
			mambo.fly_direct(roll=5, pitch=pitch, yaw=0, vertical_movement=0, duration=0.2)
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
			kill_thread = 1
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
	print("Controle no eixo Y...")
	posx = posX_cm
	erro_y = y_d - posY_cm
	while(posX_cm == 0):
		posx = posX_cm
		erro_y = y_d - posY_cm

	pitch = 0
	count = 0
	vel_y = 13
	vel_y_fino = 40
	roll = 5
	range_bias = 7
	
	while ((abs(erro_y)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0

			roll = bias_roll2(posx, math.copysign(1,ang))
			
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
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.1)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_y: %.2f" % erro_y, \
			" pitch: ", pitch, " roll: ", roll, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
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
			mambo.fly_direct(roll=5, pitch=pitch, yaw=0, vertical_movement=0, duration=0.3)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	print("done!\n")
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)

def controleDiag(x_d,y_d,ang_d,tol):
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
	count = 0
	vel_y = 13
	vel_y_fino = 40
	roll = 4
	range_bias = 7

	while ((abs(erro_y)>tol) or (abs(erro_x)>tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0

			bias_roll3(posx, posy, ang_d)

			if(abs(ang_d - ang)>tol_ang) :
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				correct_yaw(ang_d,tol_yaw) 
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
					#pitch = vel_y
					pitch = math.copysign(vel_y,erro_y*ang)
			else:
				#pitch = vel_y
				pitch = math.copysign(vel_y,erro_y*ang)
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.1)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm, " erro_x: %.2f" % erro_x, " erro_y: %.2f" % erro_y, \
			" pitch: ", pitch, " roll: ", roll, " vel_x: %.3f" %mambo.sensors.speed_x, " vel_y: %.3f" %mambo.sensors.speed_y)
			erro_x = x_d - posX_cm
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
	correct_yaw(ang_d,tol_yaw)
	#print("Controle fino diagonal...")
	#while(abs(y_d-posY_cm)>tol):
	#		pitch = math.copysign(vel_y_fino,ang*(y_d-posY_cm))
	#		mambo.fly_direct(roll=5, pitch=pitch, yaw=0, vertical_movement=0, duration=0.3)
	#		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	#		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
	print("done!\n")
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)

def altitude(h,vel):
	count = 0
	tol=2.5
	tol = 7
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
	global posX_cm, posY_cm, posZ_cm, ang, obj_x, obj_y, obj_z
	posX_cm = data.data[0]; posY_cm = data.data[1]; posZ_cm = data.data[2]; ang = data.data[3]
	obj_x = data.data[8]; obj_y = data.data[9]; obj_z = data.data[10]
	
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
		
posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
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
#success = 1;
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
		#print(wp)
		
		tol = 4 #tolerancia de waypoint (quadrada no momento para controle45, nao circular)
		tol_yaw = 4 #tol_yaw é tolerancia para correçao do angulo
		tol_ang = 12 #tol_ang é a tolerancia de erro de ang para acontecer correct_yaw
		
		#altitude(50,12)
		
		# print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		# posx = posX_cm
		# while(posY_cm > 100):
			# bias = bias_roll(posx,-1) 
			# mambo.fly_direct(roll=bias, pitch=10, yaw=0, vertical_movement=0, duration=0.1)
			# print("bias: %d" % bias)
			# print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm)
		#controleY(y, tol)
		
		# altitude(57,12)
		
		# teste de roll
		base = 0
		vel = 20
		dur = 2.5
		print("waiting 2s...")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=dur)
		print("after waiting:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("roll right 10...")
		mambo.fly_direct(roll=vel+base, pitch=0, yaw=0, vertical_movement=0, duration=dur)
		print("after roll right:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("waiting 2s...")
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=dur)
		print("after waiting:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("roll left -10...")
		mambo.fly_direct(roll=-vel+base, pitch=0, yaw=0, vertical_movement=0, duration=dur)
		print("after roll left:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("waiting 2s...")
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=dur)
		print("after waiting:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		
		# teste de pitch
		# base = -10
		print("pitch front 10...")
		mambo.fly_direct(roll=base, pitch=vel, yaw=0, vertical_movement=0, duration=dur)
		print("after:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("waiting 2s...")
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=dur)
		print("after waiting:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("pitch back -10...")
		mambo.fly_direct(roll=base, pitch=-vel, yaw=0, vertical_movement=0, duration=dur)
		print("after:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("waiting 2s...")
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=dur)
		print("after waiting:")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		# mambo.smart_sleep(2)
		# mambo.close_claw()
		# mambo.smart_sleep(2)
		# mambo.fly_direct(roll=-15, pitch=15, yaw=0, vertical_movement=0, duration=3)
		# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=2)
		# mambo.open_claw()
		# mambo.smart_sleep(2)
		#altitude(50,12)
		
		#mambo.smart_sleep(2)
		#mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
		
		# while(posZ_cm > 35):
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-5, duration=0.1)
		# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
		# mambo.close_claw()
		# mambo.smart_sleep(2)
		
		# while(posZ_cm > 30):
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-20, duration=None)
			
		# altitude(30,20)
		
		print("going down to land in 2 seconds...")
		#mambo.smart_sleep(3)
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-20, duration=2)
		
		
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
#!/usr/bin/env python3

from __future__ import print_function
from time import sleep
from threading import Thread
import math
from pyparrot.Minidrone import Mambo
import sys

import rospy
from std_msgs.msg import Float32MultiArray

def bias_roll(posx, dir): #dir=1 -> X, Y+ / dir=-1 -> X, Y- / dir=2 -> Y, X+ / dir=-2 -> Y, X-
	roll_b = 5
	global posX_cm, posY_cm
	# if(mambo.sensors.speed_y<-0.01):
		# roll_b = 10
	# if(mambo.sensors.speed_y>0.01):
		# roll_b = 1
		
	if dir == 1 or dir == -1:
		pos = posX_cm
	elif dir == 2 or dir == -2:
		pos = posY_cm
	
	if(abs(dir)==2):
		dir = -dir/2
	
	if(pos<(-5+posx)):
		roll_b = 5 + 7*dir
	if(pos>(5+posx)):
		roll_b = 5 - 7*dir
		
	if(pos<(-15+posx)):
		roll_b = 5 + 25*dir
	if(pos>(15+posx)):
		roll_b = 5 - 25*dir
	
	# if(pos<(-8+posx)):
		# roll_b = 5 + 15*dir
	# if(pos>(8+posx)):
		# roll_b = 5 - 15*dir
		
	# if(pos<(-20+posx)):
		# roll_b = 5 + 25*dir
	# if(pos>(20+posx)):
		# roll_b = 5 - 25*dir
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

def correct_yaw(base,tol):
	#global ang
	count = 0
	vel = 10
	print("corrigindo yaw...")
	while(ang==0):
		count = count + 1
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

	if(erro_x<0):
		dir = -2
	else:
		dir = 2
	while (((abs(erro_x)>tol) and (erro_x/dir > 0)) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			roll = bias_roll2(posy, dir)
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
					pitch = math.copysign(1,erro_x/dir)*10
			else:
				pitch = math.copysign(1,erro_x/dir)*10
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
			kill_thread = 1
			break
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
	while(abs(x_d-posX_cm)>tol):
			pitch = math.copysign(50,dir*(x_d-posX_cm))
			mambo.fly_direct(roll=5, pitch=pitch, yaw=0, vertical_movement=0, duration=0.2)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)

def controleXroll(x_d,tol):
	print("Controle no eixo X...")
	erro_x = x_d - posX_cm
	while(posX_cm == 0):
		erro_x = x_d - posX_cm
		
	roll = 0
	vel_x = 15
	vel_x_fino = 30
	count = 0
		
	while ((abs(erro_x)>2*tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0
			if(abs(ang)<75 or abs(ang)>105):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				correct_yaw(math.copysign(90,ang),tol_yaw)
			if(abs(mambo.sensors.speed_y) > 0.21):
				roll = 0
				print("slowing...")
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				mambo.flat_trim()
			elif(abs(erro_x) < 18):
				if(abs(mambo.sensors.speed_y) > 0.1):
					roll = 0
					print("slowing close...")
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
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
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
	correct_yaw(math.copysign(90,ang),tol_yaw)
	print("Controle fino em X (roll)...")
	while(abs(x_d-posX_cm)>tol):
		if(posX_cm != 0):
			count = 0
			roll = math.copysign(vel_x_fino,ang*(x_d-posX_cm))
			mambo.fly_direct(roll=roll, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.2)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		else:
			print("camera missed")
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
			count = count + 1
			if(count>13):
				print("count exceded! landing...")
				print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
				mambo.safe_land(5)
				mambo.smart_sleep(1)
				print("disconnect")
				mambo.disconnect()
				break
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
	bias = 1
	count = 0
	vel_y = 15
	vel_y_fino = 30
	roll = 5
	range_bias = 7
	tol_yaw = 5
	
	while ((abs(erro_y)>2*tol) or (posX_cm == 0)):
		if(posX_cm != 0):
			count = 0

			#roll = bias_roll2(posx, math.copysign(1,ang))
			#correção de bias em X
			if(posX_cm<(-range_bias+posx) or posX_cm>(range_bias+posx)):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
				while(posX_cm<(-range_bias+posx) or posX_cm>(range_bias+posx)):
					if(posX_cm == 0):
						break
					if(abs(ang)<77 or abs(ang)>103):
						mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
						correct_yaw(math.copysign(90,ang),tol_yaw)
					bias = math.copysign(25,(posX_cm-posx)*(-ang))
					print("bias: ", bias)
					mambo.fly_direct(roll=bias, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
				roll = roll + math.copysign(1,bias)
				mambo.flat_trim()
			
			if(abs(ang)<77 or abs(ang)>103):
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
				correct_yaw(math.copysign(90,ang),tol_yaw)
			if(abs(mambo.sensors.speed_x) > 0.27):
				pitch = 0
				print("slowing...")
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
				mambo.flat_trim()
			elif(abs(erro_y) < 23):
				if(abs(mambo.sensors.speed_x) > 0.12):
					pitch = 0
					print("slowing close...")
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
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
				mambo.safe_land(5)
				mambo.smart_sleep(1)
				print("disconnect")
				mambo.disconnect()
				break
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.1)
	correct_yaw(math.copysign(90,ang),tol_yaw)
	print("Controle fino em Y...")
	while(abs(y_d-posY_cm)>tol):
		if(posX_cm != 0):
			count = 0
			pitch = math.copysign(vel_y_fino,ang*(y_d-posY_cm))
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.3)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.2)
			print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
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
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
	
def altitude(h,vel):
	count = 0; count2 = 0
	tol=2
	print("going to altitude %.1f centimeters..." % h)
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
	while(abs(posZ_cm-h)>tol):
		while ((posZ_cm>h+tol or posX_cm == 0) or count < 4):
			if(posZ_cm<=h+tol):
				count = count + 1
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.3)
				print("ok")
			else:
				count = 0
				if(posX_cm == 0):
					print("camera missed")
					mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.2)
					count2 = count2 + 1
					if(count2>13):
						print("count exceded! landing...")
						print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
						mambo.safe_land(5)
						mambo.smart_sleep(1)
						print("disconnect")
						mambo.disconnect()
						break
				else:
					count2 = 0
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

def drone_node():
	rospy.init_node('drone_node', anonymous=True, disable_signals=True)
	rospy.Subscriber("camera_topic", Float32MultiArray, callback)

posX_cm = 0; posY_cm = 0; posZ_cm = 0; ang = 0
obj_x = 0; obj_y = 0; obj_z = 0;
drone_node()
mamboAddr = "D0:3A:5E:20:E6:21"
mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if(success):
	try:
		mambo.ask_for_state_update()
		mambo.smart_sleep(0.5)
		
		print("taking off!")
		mambo.safe_takeoff(5)
		mambo.smart_sleep(1)
		print("battery: ", mambo.sensors.battery)
		
		max_tilt = 30
		mambo.set_max_tilt(max_tilt)
		
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm , " ang: %.1f\n" % ang)

		# WAYPOINT
		# x_d = obj_x - 3 #108.5
		# y_d = obj_y #110
		# z_d = obj_z + 7
		x_d = 108 #108.5
		y_d = 111.5 + math.copysign(3,-ang) #dY entre garra e centro do drone
		z_d = 26 + 7 #dH entre garra e centro do drone
		tol = 2.3 #tolerancia de waypoint em cm
		tol_yaw = 5 #tolerancia de correção de yaw em graus
		
		print("WP: ",x_d," ",y_d," ",z_d,"\n")
		
		altitude(50,10)
		erro_y = y_d - posY_cm
		correct_yaw(math.copysign(90,erro_y),tol_yaw)
		controleY(y_d,tol)
	
		print("arrived pos_y")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		mambo.smart_sleep(2)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )

		altitude(50,10)
		erro_x = x_d - posX_cm
		correct_yaw(math.copysign(90,ang),tol_yaw) #mantem orientaçao anterior, pois usará controle em roll
		controleXroll(x_d,tol)
		
		print("arrived pos_x")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		mambo.smart_sleep(2)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		
		erro_x = x_d - posX_cm
		erro_y = y_d - posY_cm
		while(abs(erro_x)>tol or abs(erro_y)>tol):
				altitude(50,10)
				correct_yaw(math.copysign(90,ang),tol_yaw)
				if(abs(erro_x)>tol):
					controleXroll(x_d,tol)
				if(abs(erro_y)>tol):
					controleY(y_d,tol)
				erro_x = x_d - posX_cm
				erro_y = y_d - posY_cm
				mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1.3)
			
		print("arrived pos final")
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		# mambo.smart_sleep(1)
		# print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )

		print("going down to z_d...")
		while(abs(posZ_cm-z_d)>2):
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-5, duration=0.1)
		# altitude(z_d,6)
		print("done!")
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.4)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		print("closing claw...")
		mambo.close_claw()
		mambo.smart_sleep(2)
		
		altitude(50,10)
		
		mambo.fly_direct(roll=20, pitch=0, yaw=0, vertical_movement=0, duration=2)
		mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=2)
		
		mambo.open_claw()
		mambo.smart_sleep(2)
		print("posx: %.2f" % posX_cm , " posy: %.2f" % posY_cm , "posz: %.2f" % posZ_cm )
		
	except:
		print("abortado: ", sys.exc_info()[0])
	
	print("going down to land...")
	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-20, duration=1.5)
	print("landing")
	mambo.safe_land(5)
	print("battery: ", mambo.sensors.battery)
	
	print("disconnect")
	mambo.disconnect()
	
	#EOF
else:
	print("connection problem, exiting")
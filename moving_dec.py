"""
Demo the direct flying for the python interface
Author: Amy McGovern
"""

from pyparrot.Minidrone import Mambo

#correcao de yaw
def correct_yaw():
	(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
	mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
	if(Z>1):
		print("corrigindo yaw...")
		while Z>1:
			mambo.fly_direct(roll=0, pitch=0, yaw=-1, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("new yaw: ", Z)
		print("done adjusting yaw!")
	if(Z<-1):
		print("corrigindo yaw...")
		while Z<-1:
			mambo.fly_direct(roll=0, pitch=0, yaw=1, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("new yaw: ", Z)
		print("done adjusting yaw!")
	return

#correcao de bias para drone 1
def bias_roll():
	roll_b = 0
	if(mambo.sensors.posy<-8):
		roll_b = 10
	if(mambo.sensors.posy>8):
		roll_b = -7
	return roll_b

mamboAddr = "90:3a:e6:21:5e:20"

mambo = Mambo(mamboAddr, use_wifi=True)

print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):

	try:
		# get the state information
		print("sleeping and getting state information...")
		mambo.ask_for_state_update()
		mambo.smart_sleep(0.5)

		# TAKE OFF
		print("taking off!")
		mambo.safe_takeoff(5)
		mambo.smart_sleep(1)		
	
		max_tilt = 10
		mambo.set_max_tilt(max_tilt)

		correct_yaw()
		
		#WAYPOINT
		x_d = -40
		y_d = -30
		tol = 8 #tolerancia de erro em cm
		
		#CONTROLE EM X:
		print("Controle no eixo X...")
		erro_x = x_d - mambo.sensors.posx
		pitch = 0
		bias = 5 #bias, movimento tende para esquerda

		while abs(erro_x)>tol:
			roll = bias_roll() + bias
			erro_x = x_d - mambo.sensors.posx
			speed_x = mambo.sensors.speed_x
			speed_y = mambo.sensors.speed_y
			if((erro_x/abs(erro_x))*(speed_x/abs(speed_x))!=1): #esta indo em direcao ao aumento do erro
				pitch = 20*(erro_x/abs(erro_x))
			else:
				if(abs(speed_x) < abs(erro_x)/300):
					pitch = 8*(erro_x/abs(erro_x))
				else:	
					pitch = -1
					print("slowing...")
			
			print("posx: %.2f" % mambo.sensors.posx , " posy: %.2f" % mambo.sensors.posy, " erro_x: %.2f" % erro_x, \
			" pitch: ", pitch, " speed_x: %.2f" % speed_x, " roll: ", roll, " speed_y: %.2f" % speed_y)
			#print(" ")
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.3)
			mambo.smart_sleep(0.2)
		
		print("arrived pos_x")
		print("")
		mambo.smart_sleep(2)
		
		#GIRO DE YAW EM 90 GRAUS
		# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		# print("yaw: ", Z)
		# mambo.smart_sleep(0.5)
		# print("giro de 90 graus...")
		# if(Z>91):
			# while Z>1:
				# mambo.fly_direct(roll=0, pitch=0, yaw=-3, vertical_movement=0, duration=0.1)
				# mambo.smart_sleep(0.4)
				# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
				# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			# print("yaw corrigido: ", Z)
		# if(Z<89):
			# while Z<89:
				# mambo.fly_direct(roll=0, pitch=0, yaw=max(3,(90-Z)/8), vertical_movement=0, duration=0.1)
				# mambo.smart_sleep(0.4)
				# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
				# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			# print("yaw corrigido: ", Z)
		# mambo.smart_sleep(0.5)
		
		correct_yaw()
		print("")
		mambo.flat_trim()
		
		#CONTROLE EM Y:
		print("Controle no eixo Y...")
		erro_y = y_d - mambo.sensors.posy
		pitch = 0
		while abs(erro_y)>tol:
			erro_x = x_d - mambo.sensors.posx
			if(abs(erro_x)>tol/2):
				pitch=2*(abs(erro_x)/erro_x)
			else:
				pitch = 0
			erro_y = y_d - mambo.sensors.posy
			speed_x = mambo.sensors.speed_x
			speed_y = mambo.sensors.speed_y
			if((erro_y/abs(erro_y))*(speed_y/abs(speed_y))!=1): #esta indo em direcao ao aumento do erro
				roll = 8*(erro_y/abs(erro_y)) + bias
			else:
				if(abs(speed_y)<abs(erro_y)/250):
					roll = 5*(erro_y/abs(erro_y)) + bias
				else:	
					roll = 0 + bias
					print("slowing...")
			
			print("posx: %.2f" % mambo.sensors.posx , " posy: %.2f" % mambo.sensors.posy, " erro_y: %.2f" % erro_y, \
			" pitch: ", pitch, " speed_x: %.2f" % speed_x, " roll: ", roll, " speed_y: %.2f" % mambo.sensors.speed_y)
			#print(" ")
			mambo.fly_direct(roll=roll, pitch=pitch, yaw=0, vertical_movement=0, duration=0.3)
			mambo.smart_sleep(0.3)
		
		print("arrived pos_y")
		print("")
		mambo.smart_sleep(2)
			
		# SENSORES
		#print("altitude: ", mambo.sensors.altitude)
		print("position X: ", mambo.sensors.posx)
		print("position Y: ", mambo.sensors.posy)
		print("position Z: ", mambo.sensors.posz)
		print("")
		#(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		#mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		#print("roll: ", X)
		#print("pitch: ", Y)
		#print("yaw: ", Z)
		mambo.smart_sleep(1)
	
		# MOVING UP
		#while mambo.sensors.altitude<0.8:
		#	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=5, duration=None)
		#	mambo.smart_sleep(0.1)
	
		# OTHER MOVE COMMANDS:
		#mambo.turn_degrees(90)
		
	except:
		print("-- CANCELADO!!! --")

	print("landing")
	mambo.safe_land(5)
	mambo.smart_sleep(1)

	print("disconnect")
	mambo.disconnect()
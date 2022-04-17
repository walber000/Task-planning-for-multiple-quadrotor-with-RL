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
			mambo.fly_direct(roll=0, pitch=0, yaw=-2, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("new yaw: ", Z)
		print("done adjusting yaw!")
	if(Z<-1):
		print("corrigindo yaw...")
		while Z<-1:
			mambo.fly_direct(roll=0, pitch=0, yaw=2, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("new yaw: ", Z)
		print("done adjusting yaw!")
	return

#correcao de bias para drone 1
def bias_roll():
	roll_b = 0
	if(mambo.sensors.posy<-15):
		roll_b = 10
	if(mambo.sensors.speed_y<-0.01):
		roll_b = 10
	return roll_b
		
# you will need to change this to the address of YOUR mambo
mamboAddr = "90:3a:e6:21:5e:20"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
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
		mambo.smart_sleep(2)		
	
		max_tilt = 10
		mambo.set_max_tilt(max_tilt)
		
		correct_yaw()
	
		# MOVING DOWN
		#print("going down to altitude 0.5 meters...")
		#while mambo.sensors.altitude>0.5:
		#	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-7, duration=None)
		#	mambo.smart_sleep(0.1)
		#print("done! altitude: ", mambo.sensors.altitude)
		#mambo.smart_sleep(1)
		
		#CONTROLE PROPORCIONAL BASEADO NO ERRO DE POSICAO
		#MOVING TO POSITION
		#x_d = 0
		#y_d = 30
		#print("going ahead to position x= ", x_d, " cm, y= ", y_d, " cm")
		#erro_x = x_d - mambo.sensors.posx
		#erro_y = y_d - mambo.sensors.posy
		#K_x = 0.05
		#K_y = 0.05
		#tol = 4 #tolerancia de erro em cm
		#while abs(erro_x)>tol or abs(erro_y)>tol:
		#	erro_x = x_d - mambo.sensors.posx
		#	erro_y = y_d - mambo.sensors.posy
		#	if(abs(erro_x)<tol):
		#		pitch = 0
		#	else:
		#		if(erro_x>0):
		#			pitch = max(K_x*erro_x, 1)
		#		else:
		#			pitch = min(K_x*erro_x, -1)
		#	if(abs(erro_y)<tol):
		#		roll = 0
		#	else:
		#		if(erro_y>0):
		#			roll = max(K_y*erro_y, 1)
		#		else:
		#			roll = min(K_y*erro_y, -1)
		#	mambo.fly_direct(roll, pitch, yaw=0, vertical_movement=0, duration=None)
		#	print("posx: ", mambo.sensors.posx , " posy: ", mambo.sensors.posy)
		#	print("erro_x: ", erro_x , " erro_y: ", erro_y)
		#	print("pitch: ", pitch , " roll: ", roll)
		#	mambo.smart_sleep(0.1)
		
		#CONTROLE PROPORCIONAL BASEADO NO ERRO DE POSICAO E NA VELOCIDADE
		#MOVING TO POSITION
		x_d = 0
		y_d = 40
		print("going ahead to position x= ", x_d, " cm, y= ", y_d, " cm")
		erro_x = x_d - mambo.sensors.posx
		erro_y = y_d - mambo.sensors.posy
		#K1_x = 0.2
		#K1_y = 0.2
		#K2_x = 200
		#K2_y = 200
		roll=0
		pitch=0
		tol = 5 #tolerancia de erro em cm
		while abs(erro_x)>tol or abs(erro_y)>tol:
			erro_x = x_d - mambo.sensors.posx
			erro_y = y_d - mambo.sensors.posy
			#pitch = K1_x*(erro_x - K2_x*mambo.sensors.speed_x)
			#roll  = K1_y*(erro_y - K2_y*mambo.sensors.speed_y) + 2.2
			
			if((erro_x/abs(erro_x))*(mambo.sensors.speed_x/abs(mambo.sensors.speed_x))!=1): #esta indo em direcao ao aumento do erro
				pitch = erro_x/3
			else:
				if(abs(mambo.sensors.speed_x)<0.1):
					pitch = erro_x/6
				else:	
					pitch = erro_x/15
			if(abs(pitch)<0.6):
				pitch=0
			
			if((erro_y/abs(erro_y))*(mambo.sensors.speed_y/abs(mambo.sensors.speed_y))!=1): #esta indo em direcao ao aumento do erro
				roll = (erro_y/3) + 2.8
			else:
				if(abs(mambo.sensors.speed_y)<0.1):
					roll = (erro_y/6) + 2.8
				else:	
					roll = (erro_y/15) + 2.8
			if(abs(roll)<3.3):
				roll=2.8
			
			print("posx: ", mambo.sensors.posx , " posy: ", mambo.sensors.posy)
			print("erro_x: ", erro_x , " erro_y: ", erro_y)
			print("pitch: ", pitch , " roll: ", roll)
			print(" ")
			mambo.fly_direct(roll, pitch, yaw=0, vertical_movement=0, duration=0.3)
			mambo.smart_sleep(0.3)
		
		print("arrived!")
		mambo.smart_sleep(2)
			
		# SENSORES
		#print("altitude: ", mambo.sensors.altitude)
		print("position X: ", mambo.sensors.posx)
		print("position Y: ", mambo.sensors.posy)
		print("position Z: ", mambo.sensors.posz)
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
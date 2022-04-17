"""
Demo the direct flying for the python interface
Author: Amy McGovern
"""

from pyparrot.Minidrone import Mambo

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
		print("sleeping")
		mambo.ask_for_state_update()
		mambo.smart_sleep(0.5)

		print("taking off!")
		mambo.safe_takeoff(5)
		
		mambo.set_max_tilt(10)
		
		# MOVING DOWN
		#print("going down to altitude 0.5 meters...")
		#while mambo.sensors.altitude>0.5:
		#	mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-7, duration=None)
		#	mambo.smart_sleep(0.1)
		#print("done! altitude: ", mambo.sensors.altitude)

		# CORREÇÃO YAW
		(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		print("yaw: ", Z, "   psi: ", mambo.sensors.psi)
		mambo.smart_sleep(0.5)
		print("corrigindo yaw...")
		if(Z>1):
			while Z>1:
				mambo.fly_direct(roll=0, pitch=0, yaw=-2, vertical_movement=0, duration=0.1)
				mambo.smart_sleep(0.4)
				(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
				mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("yaw corrigido: ", Z, "   psi: ", mambo.sensors.psi)
		if(Z<-1):
			while Z<-1:
				mambo.fly_direct(roll=0, pitch=0, yaw=2, vertical_movement=0, duration=0.1)
				mambo.smart_sleep(0.4)
				(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
				mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("yaw corrigido: ", Z, "   psi: ", mambo.sensors.psi)
		mambo.smart_sleep(0.5)
		
		print("TESTE 1")
		for i in range(2):
			mambo.fly_direct(roll=0, pitch=10, yaw=0, vertical_movement=0, duration=5)
			print("x speed: ", mambo.sensors.speed_x)
			print("y speed: ", mambo.sensors.speed_y)
			print("position X: ", mambo.sensors.posx)
			print("position Y: ", mambo.sensors.posy)
			print("yaw: ", Z, "   psi: ", mambo.sensors.psi)
		print("DONE!")
		mambo.smart_sleep(4)
		
		print("position X: ", mambo.sensors.posx)
		print("position Y: ", mambo.sensors.posy)
		print("yaw: ", Z, "   psi: ", mambo.sensors.psi)
		
		#FLYING FORWARD
		#print("Flying direct: going forward (positive pitch)")
		#while(mambo.sensors.posx<30):
		#	mambo.fly_direct(roll=2, pitch=1, yaw=0, vertical_movement=0, duration=None)
		#	print("x speed: ", mambo.sensors.speed_x)
		#	print("y speed: ", mambo.sensors.speed_y)
		#	print(" ")
		#	mambo.smart_sleep(0.1)
			
		#print("x speed: ", mambo.sensors.speed_x)
		#print("position X: ", mambo.sensors.posx)
		#mambo.smart_sleep(3)
		#print("x speed: ", mambo.sensors.speed_x)
		#print("position X: ", mambo.sensors.posx)

	except:
		print("-- CANCELADO!!! --")

	print("landing")
	mambo.safe_land(5)
	mambo.smart_sleep(1)

	print("disconnect")
	mambo.disconnect()
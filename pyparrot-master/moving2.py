"""
Demo the direct flying for the python interface
Author: Amy McGovern
"""

from pyparrot.Minidrone import Mambo
from threading import Thread
from time import sleep

# kill_thread = 0

# class status(Thread):
	# def __init__(self):
		# Thread.__init__(self)
		
	# def run(self):
		# while(kill_thread!=1):
			# print(" STATE: ",mambo.sensors.flying_state)
			# sleep(0.4)

#correcao de yaw
def correct_yaw():
	(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
	mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
	print("corrigindo yaw...")
	if(Z>1):
		while Z>1:
			mambo.fly_direct(roll=0, pitch=0, yaw=-1, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("yaw: ", Z)
			print("psi: ", mambo.sensors.psi)
		print("done adjusting yaw!")
	if(Z<-1):
		while Z<-1:
			mambo.fly_direct(roll=0, pitch=0, yaw=1, vertical_movement=0, duration=0.1)
			mambo.smart_sleep(0.4)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
			mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("yaw: ", Z)
			print("psi: ", mambo.sensors.psi)
	print("done adjusting yaw!")
	mambo.smart_sleep(2)
	print("yaw: ", Z)
	print("psi: ", mambo.sensors.psi)
	return

#correcao de bias para drone 1
def bias_roll():
	roll_b = 5
	if(mambo.sensors.speed_y<-0.01):
		roll_b = 15
	if(mambo.sensors.speed_y>0.01):
		roll_b = -10
	if(mambo.sensors.posy<-10):
		roll_b = 15
	if(mambo.sensors.posy>10):
		roll_b = -10
	return roll_b
		

# you will need to change this to the address of YOUR mambo
#mamboAddr = "90:3a:e6:21:5e:20"
mamboAddr = "D0:3A:5E:20:E6:21"
mamboAddr = "D0:3A:BD:55:E6:23" #segundo drone


# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=False)

print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):
	try:
		# get the state information
		print("sleeping")
		mambo.ask_for_state_update()
		mambo.smart_sleep(0.5)

		# info = status()
		# info.start()
		
		print("taking off!")
		mambo.safe_takeoff(5)
		mambo.smart_sleep(1)
		
		max_tilt = 10
		mambo.set_max_tilt(max_tilt)

		# correct_yaw()
		
		# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		# print("yaw: ", Z)
		# print("")
		
		# mambo.flat_trim()
		# print("flat trim...")
		# mambo.smart_sleep(3)
		
		# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		# print("yaw: ", Z)
		# print("")
		
		#TESTE
		# print("TESTE DE GIRO")
		# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		# print("yaw: ", Z)
		# mambo.smart_sleep(3)
		# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		# print("yaw: ", Z)
		# mambo.smart_sleep(3)
		# mambo.turn_degrees(90)
		# mambo.smart_sleep(3)
		# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		# print("yaw: ", Z)
		# mambo.smart_sleep(3)
		# (X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w,
		# mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		# print("yaw: ", Z)
		
		# mambo.sensors.flying_state = "flying"
		#print(" STATE: ",mambo.sensors.flying_state)
		#mambo.smart_sleep(1)
		
		# print("one")
		# print("altitude: %.3f" % mambo.sensors.altitude)
		# print("two")
		# print("posz: %.3f" % mambo.sensors.posz)
		# print("three")
		
		# print("Showing turning (in place) using turn_degrees")
		# mambo.turn_degrees(90)
		# mambo.smart_sleep(2)
		# mambo.turn_degrees(-90)
		# mambo.smart_sleep(2)
		
		#for i in range(2):
		#MOVING DOWN
		#print("\nTESTE 1\n")
		#print("going down to altitude 0.5 meters...")
		# while mambo.sensors.altitude>0.5:
			# if(mambo.sensors.altitude<0.6):
				# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-2, duration=0.5)
			# else:
				# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-4, duration=0.5)
			# print("altitude: %.3f" % mambo.sensors.altitude)
			# #print(" STATE: ",mambo.sensors.flying_state)
		# print("done!\n")
		# mambo.smart_sleep(1)
				
		# print("\nmoving...")
		# #for i in range(2):
		# mambo.fly_direct(roll=5, pitch=10, yaw=0, vertical_movement=0, duration=3)
		# print("\nstopped!")
		# mambo.smart_sleep(2)
		# mambo.flat_trim()
		# mambo.fly_direct(roll=-5, pitch=0, yaw=0, vertical_movement=0, duration=1)	
		# mambo.smart_sleep(2)
		
		# for i in range(15):
			# print("altitude: %.3f" % mambo.sensors.altitude)
			# #print(" STATE: ",mambo.sensors.flying_state)
			# mambo.smart_sleep(1)
			
		# mambo.sensors.flying_state = "hovering"
		# #print(" STATE: ",mambo.sensors.flying_state)
		# mambo.smart_sleep(1)
		
		# for i in range(10):
			# print("altitude: %.3f" % mambo.sensors.altitude)
			# #print(" STATE: ",mambo.sensors.flying_state)
			# mambo.smart_sleep(1)
			
			# print("\nTESTE 2\n")
			# while mambo.sensors.altitude<0.9:
				# if(mambo.sensors.altitude>0.8):
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=2, duration=None)
					# mambo.smart_sleep(0.2)
				# else:
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=4, duration=None)
					# mambo.smart_sleep(0.2)
				# print("altitude: %.3f" % mambo.sensors.altitude)	
			# print("done!\n")
			# mambo.smart_sleep(1)
			# for i in range(5):
				# print("altitude: %.3f" % mambo.sensors.altitude)
				# mambo.smart_sleep(1)
		
		# for i in range(3):
			# # MOVING DOWN
			# print("\nTESTE 1\n")
			# #print("going down to altitude 0.5 meters...")
			# while mambo.sensors.altitude>0.5:
				# if(mambo.sensors.altitude<0.6):
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-2, duration=None)
					# mambo.smart_sleep(0.2)
				# else:
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-4, duration=None)
					# mambo.smart_sleep(0.2)
				# print("altitude: %.3f" % mambo.sensors.altitude)	
			# print("done!\n")
			# mambo.smart_sleep(1)
			# for i in range(8):
				# print("altitude: %.3f" % mambo.sensors.altitude)
				# mambo.smart_sleep(1)
				
			# print("\nTESTE 2\n")
			# #print("going down to altitude 0.5 meters...")
			# while mambo.sensors.altitude<0.9:
				# if(mambo.sensors.altitude>0.8):
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=2, duration=None)
					# mambo.smart_sleep(0.2)
				# else:
					# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=4, duration=None)
					# mambo.smart_sleep(0.2)
				# print("altitude: %.3f" % mambo.sensors.altitude)	
			# print("done!\n")
			# mambo.smart_sleep(1)
			# for i in range(8):
				# print("altitude: %.3f" % mambo.sensors.altitude)
				# mambo.smart_sleep(1)
				
		# print("")
		# for i in range(15):
			# print("altitude: %.3f" % mambo.sensors.altitude)
			# mambo.smart_sleep(1)
		
		# print("")
		# for i in range(10):
			# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=1)
			# print("altitude: %.3f" % mambo.sensors.altitude)
			# mambo.smart_sleep(2)
		
		# while(mambo.sensors.posx<80):
			# speed_x = mambo.sensors.speed_x
			# roll = bias_roll() + 5
			# if(speed_x < 0.1):
				# mambo.fly_direct(roll=roll, pitch=8, yaw=0, vertical_movement=0, duration=0.5)
			# else:
				# print("  slow...  ")
				# mambo.fly_direct(roll=roll, pitch=-1, yaw=0, vertical_movement=0, duration=0.5)
				# mambo.smart_sleep(1)
				# mambo.flat_trim()
			
			# print("roll: ", roll, "x_spd: %.2f" % mambo.sensors.speed_x, "y_spd: %.2f" % mambo.sensors.speed_y, "posy: %.2f" % mambo.sensors.posy)

		# print("ARRIVED!!!")
		# mambo.smart_sleep(1.5)
		# print("position X: %.2f" % mambo.sensors.posx)
		# print("position Y: %.2f" % mambo.sensors.posy)
		# print("")
		
		#FLYING FORWARD
		#print("Flying direct: going forward (positive pitch)")
		#while(mambo.sensors.posx<30):
		#	mambo.fly_direct(roll=2, pitch=1, yaw=0, vertical_movement=0, duration=None)
		#	print("x speed: ", mambo.sensors.speed_x)
		#	print("y speed: ", mambo.sensors.speed_y)
		#	print(" ")
		#	mambo.smart_sleep(0.1)
		
		#print("Showing turning (in place) using turn_degrees")
		#mambo.turn_degrees(90)
		#mambo.smart_sleep(2)
		#mambo.turn_degrees(-90)
		#mambo.smart_sleep(2)

	except:
		print("-- CANCELADO!!! --")

	print("landing")

	# kill_thread = 1
	
	# MOVING DOWN
	# print("going down to altitude 0.5 meters...")
	# while mambo.sensors.altitude>0.5:
		# mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-15, duration=None)
		# mambo.smart_sleep(0.1)
	# print("done! altitude: %.2f" % mambo.sensors.altitude)
	# print("position X: %.2f" % mambo.sensors.posx)
	# print("position Y: %.2f" % mambo.sensors.posy)
	mambo.safe_land(5)
	mambo.smart_sleep(1)
	
	print("disconnect")
	mambo.disconnect()
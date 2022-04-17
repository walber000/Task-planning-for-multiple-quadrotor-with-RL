"""
Demo the direct flying for the python interface
Author: Amy McGovern
"""

from pyparrot.Minidrone import Mambo
from time import sleep
from threading import Thread

# you will need to change this to the address of YOUR mambo
# mamboAddr = "90:3a:e6:21:5e:20"
# mamboAddr = "D0:3A:5E:20:E6:21"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
# mambo = Mambo(mamboAddr, use_wifi=False)

# print("trying to connect")
# success = mambo.connect(num_retries=3)
# print("connected: %s" % success)

def bias_roll(posy):
	roll_b = 5
	if(mambo.sensors.speed_y<-0.01):
		roll_b = 15
	if(mambo.sensors.speed_y>0.01):
		roll_b = -10
	return roll_b

class drone(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self):
		try:
			global mambo
			
			mambo.ask_for_state_update()
			mambo.smart_sleep(0.5)
			
			# SENSORES
			print("Battery level: ", mambo.sensors.battery)
			print("flying state: ", mambo.sensors.flying_state)
			print("x speed: ", mambo.sensors.speed_x)
			print("y speed: ", mambo.sensors.speed_y)
			print("z speed: ", mambo.sensors.speed_z)
			print("altitude: ", mambo.sensors.altitude)
			(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w, mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
			print("roll: ", X)
			print("pitch: ", Y)
			print("yaw: ", Z)
			print("position X: ", mambo.sensors.posx)
			print("position Y: ", mambo.sensors.posy)
			print("position Z: ", mambo.sensors.posz)
		
			max_tilt = 10
			mambo.set_max_tilt(max_tilt)
			mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=1, duration=1)
		
			rollb = bias_roll(0)
			print(rollb)
			
		except:
			print("abortado")
		
		print("disconnect")
		mambo.disconnect()

mamboAddr = "D0:3A:5E:20:E6:21"
mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

thread2 = drone()
thread2.start()
thread2.join()

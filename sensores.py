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
success = mambo.connect(num_retries=2)
print("connected: %s" % success)

if (success):
# get the state information

	try:
		print("sleeping and getting state information...")
		mambo.smart_sleep(2)
		mambo.ask_for_state_update()
		mambo.smart_sleep(2)

		# TAKE OFF
		#print("taking off!")
		#mambo.safe_takeoff(5)

		# SENSORES
		#print("Battery level: ", mambo.sensors.battery)
		#print("flying state: ", mambo.sensors.flying_state)
		#print("x speed: ", mambo.sensors.speed_x)
		#print("y speed: ", mambo.sensors.speed_y)
		#print("z speed: ", mambo.sensors.speed_z)
		#print("altitude: ", mambo.sensors.altitude)
		(X, Y, Z) = mambo.sensors.quaternion_to_euler_angle(mambo.sensors.quaternion_w, mambo.sensors.quaternion_x, mambo.sensors.quaternion_y, mambo.sensors.quaternion_z)
		print("roll: ", X)
		print("pitch: ", Y)
		print("yaw: ", Z)
		print("position X: ", mambo.sensors.posx)
		print("position Y: ", mambo.sensors.posy)
		print("position Z: ", mambo.sensors.posz)
	
		#print("ALL: ",mambo.sensors.str())
		#print("other sensors: ", mambo.sensors.sensors_dict)
	
		mambo.smart_sleep(2)
	
	except:
		print("DEU RUIM EM ALGUM LUGAR!!")
	
	#print("landing")
	#mambo.safe_land(5)
	#mambo.smart_sleep(2)
	
	print("disconnect")
	mambo.disconnect()
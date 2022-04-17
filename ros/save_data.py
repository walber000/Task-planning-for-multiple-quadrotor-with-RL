#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
import numpy as np
import math
import os.path

X = []; Y = []; Z = []
posX_cm = 0; posY_cm = 0; posZ_cm = 0
count = 1

def callback(data):
	global posX_cm, posY_cm, posZ_cm
	posX_cm = data.data[0]; posY_cm = data.data[1]; posZ_cm = data.data[2]

def listener():
	rospy.init_node('listener', anonymous=True, disable_signals=True)
	rospy.Subscriber("camera_topic", Float32MultiArray, callback)

if __name__ == '__main__':
	listener()
	try:
		i=1
		name = "data_saved%d.txt"%i
		while(os.path.isfile(name)):
			i=i+1
			name = "data_saved%d.txt"%i
		f = open(name, 'w')
		print("opening %s..."%name)
		while(1):
			X.append(posX_cm)
			Y.append(posY_cm)
			Z.append(posZ_cm)
			sleep(0.03)
	except:
		print("cancelado")

	print("saving...")
	for item in range(len(X)):		
		f.write("%.1f," % X[item])
		f.write("%.1f," % Y[item])
		f.write("%.1f\r\n" % Z[item])
	print("saved!")
	
	f.close()	
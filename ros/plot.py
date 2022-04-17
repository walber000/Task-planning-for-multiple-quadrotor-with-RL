#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import math

counter = []; X = []; Y = []; Z = []
posX_cm = 0; posY_cm = 0; posZ_cm = 0
count = 1

def animate(i):
	global count, posX_cm, posY_cm, posZ_cm
	counter.append(count)
	X.append(posX_cm)
	Y.append(posY_cm)
	Z.append(posZ_cm)
	ax1.clear()
	ax1.plot(counter, X)
	ax2.clear()
	ax2.plot(counter, Y)
	ax3.clear()
	ax3.plot(counter, Z)
	count = count + 1
	ax1.set_xlim([max(count-50,0), count])
	ax2.set_xlim([max(count-50,0), count])
	ax3.set_xlim([max(count-50,0), count])
	ax1.set_ylim([0, 170])
	ax2.set_ylim([0, 300])
	ax3.set_ylim([0, 100])

def callback(data):
	global posX_cm, posY_cm, posZ_cm
	posX_cm = data.data[0]; posY_cm = data.data[1]; posZ_cm = data.data[2]

def listener():
	rospy.init_node('listener', anonymous=True, disable_signals=True)
	rospy.Subscriber("camera_topic", Float32MultiArray, callback)

if __name__ == '__main__':
	listener()
	fig = plt.figure(figsize=(9, 3))
	ax1 = fig.add_subplot(1,3,1)
	ax2 = fig.add_subplot(1,3,2)
	ax3 = fig.add_subplot(1,3,3)
	ani = animation.FuncAnimation(fig, animate, interval=1)
	plt.show()
	#rospy.spin()
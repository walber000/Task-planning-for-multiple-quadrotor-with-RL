#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from mpl_toolkits import mplot3d

import math

counter = []; X = []; Y = []; Z = []
posX_cm = 0; posY_cm = 0; posZ_cm = 0
count = 1

def animate(i):
	global count, posX_cm, posY_cm, posZ_cm
	# counter.append(count)
	
	X.append(posX_cm)
	Y.append(posY_cm)
	Z.append(posZ_cm)
	# ax.plot3D(X, Y, Z, 'gray')
	
	ax.scatter(posX_cm, posY_cm, posZ_cm, c="red")
	#plt.hold(True)
	
	# ax1.clear()
	# ax1.plot(counter, X)
	# ax2.clear()
	# ax2.plot(counter, Y)
	# ax3.clear()
	# ax3.plot(counter, Z)
	# count = count + 1
	# ax1.set_xlim([max(count-50,0), count])
	# ax2.set_xlim([max(count-50,0), count])
	# ax3.set_xlim([max(count-50,0), count])
	# ax1.set_ylim([0, 170])
	# ax2.set_ylim([0, 300])
	# ax3.set_ylim([0, 100])
	# ax.set_xlim([0, 170])
	# ax.set_ylim([0, 300])
	# ax.set_zlim([0, 100])

def callback(data):
	global posX_cm, posY_cm, posZ_cm
	posX_cm = data.data[0]; posY_cm = data.data[1]; posZ_cm = data.data[2]

def listener():
	rospy.init_node('listener', anonymous=True, disable_signals=True)
	rospy.Subscriber("camera_topic", Float32MultiArray, callback)

if __name__ == '__main__':
	listener()
	
	# fig = plt.figure(figsize=(9, 3))
	# ax1 = fig.add_subplot(1,3,1)
	# ax2 = fig.add_subplot(1,3,2)
	# ax3 = fig.add_subplot(1,3,3)
	# ani = animation.FuncAnimation(fig, animate, interval=1)
	# plt.show()
	fig = plt.figure()
	ax = plt.axes(projection="3d")
	ax.set_xlim([0, 160])
	ax.set_ylim([0, 300])
	ax.set_zlim([0, 60])
	# ani = animation.FuncAnimation(fig, animate, interval=1)

	# z_line = np.linspace(0, 15, 1000)
	# x_line = np.cos(z_line)
	# y_line = np.sin(z_line)
	# z_points = 15 * np.random.random(100)
	# x_points = np.cos(z_points) + 0.1 * np.random.randn(100)
	# y_points = np.sin(z_points) + 0.1 * np.random.randn(100)
	# ax.scatter3D(x_points, y_points, z_points, c=z_points, cmap='hsv');

	# plt.show()
	
	
	
	try:
		f = open('plot3d.txt', 'w')
		while(1):
			ax.scatter(posX_cm, posY_cm, posZ_cm, c="red")
			# sleep(10)
			plt.pause(0.01)
			X.append(posX_cm)
			Y.append(posY_cm)
			Z.append(posZ_cm)
	except:
		print("cancelado")

	print("saving...")
	for item in range(len(X)):		
		f.write("%.1f," % X[item])
		f.write("%.1f," % Y[item])
		f.write("%.1f\r\n" % Z[item])
	print("saved!")
	
	f.close()	
	plt.show()
	
	#rospy.spin()
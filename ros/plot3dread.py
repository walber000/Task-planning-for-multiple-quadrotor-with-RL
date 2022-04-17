#!/usr/bin/env python3

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

if __name__ == '__main__':
	fig = plt.figure()
	ax = plt.axes(projection="3d")
	# ax.set_xlim([posX_cm-10, posX_cm+10])
	# ax.set_ylim([posY_cm-10, posY_cm+10])
	# ax.set_zlim([posZ_cm-10, posZ_cm+10])
	
	try:
		f = open('plot3d.txt', 'r')
		x=[];y=[];z=[]
		text_file = open("plot3d.txt", "r")
		lines = text_file.read().split('\n')
		del lines[len(lines)-1]
		for i in range(len(lines)):
			lines[i] = lines[i].split(',')
			x.append(float(lines[i][0]))
			y.append(float(lines[i][1]))
			z.append(float(lines[i][2]))
		ax.scatter(x, y, z, c="red")

	except:
		print("cancelado")
	
	f.close()	
	plt.show()
	
	#rospy.spin()
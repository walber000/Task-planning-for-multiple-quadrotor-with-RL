#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy

def callback(data):
	global i, wp
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	wp=numpy.zeros((int((len(data.data)/2)),2))
	wp[:,0]=data.data[0:int(len(data.data)/2)]
	wp[:,1]=data.data[int(len(data.data)/2):len(data.data)]
	print("wp: \n",wp)

def listener_wp():
	rospy.init_node('listener_wp', anonymous=True)
	rospy.Subscriber('wp', Float32MultiArray, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener_wp()
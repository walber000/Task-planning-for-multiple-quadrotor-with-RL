#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

def callback(data):
	rospy.loginfo("posX: %.2f" % data.data[0] + " posY: %.2f" % data.data[1] + " posZ: %.2f" % data.data[2] + " ang: %.1f", data.data[3])
	#rospy.loginfo("I heard %.1f", data.data[0])

def listener():
	rospy.init_node('listener', anonymous=True, disable_signals=True)

	rospy.Subscriber("camera_topic", Float32MultiArray, callback)

	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin()

if __name__ == '__main__':
	listener()
	try:
		for i in range(5):
			print("doing things...")
			sleep(0.5)
	except:
		print("falow")
	#rospy.spin()
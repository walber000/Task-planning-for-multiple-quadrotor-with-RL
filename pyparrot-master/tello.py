# Maneuvering
from time import sleep
from easytello import tello
from threading import Thread
import threading
import ctypes

def cada_um_no_seu_square(square_edge):
	drone.left(square_edge)
	drone.forward(square_edge)
	drone.right(square_edge)
	drone.back(square_edge)
	

def square_reverso(square_edge, speed):
	z_ref = 0

	drone.go(0, square_edge, z_ref, speed)

	sleep(1)

	drone.go(square_edge, 0, z_ref, speed)

	sleep(1)

	drone.go(0, -square_edge, z_ref, speed)

	sleep(1)

	drone.go(-square_edge, 0, z_ref, speed)

class rc_tello(Thread):
	def __init__(self):
		Thread.__init__(self)
		
	def run(self):
		try:
			global roll,pitch,vertical,yaw, movenow
			while True:
				if(movenow==1):
					drone.rc_control(roll, pitch, vertical, yaw)
				if(movenow==-1):
					drone.rc_control(0, 0, 0, 0)	
				sleep(0.1)
		except KeyboardInterrupt:
			print('Interrupt by user...')
			drone.land()
	
	def get_id(self): 
		# returns id of the respective thread 
		if hasattr(self, '_thread_id'): 
			return self._thread_id 
		for id, thread in threading._active.items(): 
			if thread is self: 
				return id	
	
	def raise_exception(self): 
		thread_id = self.get_id() 
		res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 
			  ctypes.py_object(SystemExit)) 
		if res > 1: 
			ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0) 
			print('Exception raise failure') 

def restart():
	global movenow, thread1
	thread1.raise_exception()
	sleep(0.1)
	movenow=0
	thread1 = rc_tello()
	thread1.start()

def finish():
	global thread1
	thread1.raise_exception()

drone = tello.Tello()
movenow = 0
thread1 = rc_tello()
thread1.start()

speed = 50
z_ref = 0
square_edge = 20

print(drone.get_speed())
print(drone.get_battery())

try:
	drone.takeoff()

	i=0
	
	print("run1")
	pitch=20; roll=0; vertical=0; yaw=0
	movenow=1
	sleep(2)
	restart()
	movenow=-1
	sleep(0.1)
	restart()
	
	print("run2")
	pitch=0; roll=20; vertical=0; yaw=0
	movenow=1
	sleep(2)
	restart()
	movenow=-1
	sleep(0.1)
	restart()
	
	print("run3")
	pitch=0; roll=0; vertical=20; yaw=0
	movenow=1
	sleep(2)
	restart()
	movenow=-1
	sleep(0.1)
	restart()
	
	print("run4")
	pitch=-20; roll=0; vertical=0; yaw=0
	movenow=1
	sleep(2)
	restart()
	movenow=-1
	sleep(2)
	restart()
	sleep(1)
	# thread1.kill()
	# drone.go(50, 0, 0, 10)
	# drone.wait(0.5)
	# drone.send_command('stop')

	# i=0
	# while(i<23):
		# drone.cw(4)
		# i=i+1
	# drone.wait(1)

	# i=0
	# while(i<2):
		# drone.go(0,0,20,10)
		# i=i+1
	# drone.wait(1)
except KeyboardInterrupt:
	print('STOP!')

finish()
print("land...")
drone.land()
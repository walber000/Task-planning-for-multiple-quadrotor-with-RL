# Maneuvering
from time import sleep
from threading import Thread
import threading
import ctypes

class rc_tello(Thread):
	def __init__(self):
		Thread.__init__(self)
		
	def run(self):
		try:
			global pitch
			while True:
				if(movenow):
					print("moving...%.1f"%pitch)
				sleep(0.8)
		except KeyboardInterrupt:
			print('Interrupt by user...')
			
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
	
movenow = 0
thread1 = rc_tello()
thread1.start()

try:

	pitch=20
	movenow=1
	print("run0")
	sleep(2)
	print("done0")
	restart()
	
	pitch=0
	movenow=1
	print("run1")
	sleep(2)
	print("done1")
	restart()

	pitch=10
	movenow=1
	print("run2")
	sleep(2)
	print("done2")
	restart()
	
	finish()
	print("land...")

except KeyboardInterrupt:
	# drone.emergency()
	print('STOP!')
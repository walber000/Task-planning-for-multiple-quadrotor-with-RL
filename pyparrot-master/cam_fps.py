#tutorial increase Fps: https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/
from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils

import numpy as np
import cv2
from time import sleep

import datetime
from threading import Thread
 
 		
# camera axis:
# cap = cv2.VideoCapture("http://161.24.19.89/axis-cgi/jpg/image.cgi") #imagem
# cap = cv2.VideoCapture("http://161.24.19.89/mjpg/video.mjpg") #video
# cap = cv2.VideoCapture("http://161.24.19.207:8081/img/snapshot.cgi") #imagem

# camera teto
# cap = cv2.VideoCapture("http://192.168.0.81/img/snapshot.cgi") #imagem
# cap = cv2.VideoCapture("http://192.168.0.81/img/video.mjpeg") #video

# url = "http://192.168.0.81/img/video.mjpeg" #teto
# url = "http://161.24.19.89/mjpg/video.mjpg" #axis
 
# AUTENTICAÇÃO (AINDA NAO TESTADO)
# url = "http://username:senha@161.24.19.89/mjpg/video.mjpg" #axis
		
url = "http://192.168.0.81/img/video.mjpeg" #teto
#url = "http://161.24.19.89/mjpg/video.mjpg" #axis
		
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())
		
# grab a pointer to the video stream and initialize the FPS counter
print("[INFO] sampling frames from webcam...")
stream = cv2.VideoCapture(url)
fps = FPS().start()
 
# loop over some frames
while fps._numFrames < args["num_frames"]:
	# grab the frame from the stream and resize it to have a maximum
	# width of 400 pixels
	(grabbed, frame) = stream.read()
	frame = imutils.resize(frame, width=400)
 
	# check to see if the frame should be displayed to our screen
	if args["display"] > 0:
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
 
	# update the FPS counter
	fps.update()
 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
stream.release()
cv2.destroyAllWindows()
		
		
# created a *threaded* video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src=url).start()
fps = FPS().start()
 
# loop over some frames...this time using the threaded stream
while fps._numFrames < args["num_frames"]*10:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=400)
 
	# check to see if the frame should be displayed to our screen
	if args["display"] > 0:
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
 
	# update the FPS counter
	fps.update()
 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()	

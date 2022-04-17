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

#le imagem
im = cv2.imread('test.png')

#gray scale
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

#aplica threshold
ret, thresh = cv2.threshold(imgray, 127, 255, 0)

#traca contornos
contours, hierarchy = cv2.findContours(thresh, 1, 2)

#desenha contornos em cima da imagem original
#cv2.drawContours(im, contours, -1, (0,255,0), 3)

#momentos do contorno, p.ex. centroide
cnt = contours[0]
M = cv2.moments(cnt)
cx = int(M['m10']/M['m00']) #centroide
cy = int(M['m01']/M['m00'])
print("cx = ", cx, " cy = ", cy)

#desenhar centroide
#im[(cx-5):(cx+5),(cy-5):(cy+5)]=[127,127,127]
#cv2.circle(im,(cx,cy),2,(0,0,255),3)

#area do contorno (em pixels?)
area = cv2.contourArea(cnt) # ou area = M['m00']
print("area = ", area)

#contorno com menos vertices
#epsilon = 0.1*cv2.arcLength(cnt,True)
#approx = cv2.approxPolyDP(cnt,epsilon,True)
#cv2.drawContours(im, approx, -1, (0,255,0), 3)
#hull = cv2.convexHull(cnt)
#cv2.drawContours(im, hull, -1, (0,255,0), 3)

#retangulo
x,y,w,h = cv2.boundingRect(cnt)
cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)

#menor retangulo
#rect = cv2.minAreaRect(cnt)
#box = cv2.boxPoints(rect)
#box = np.int0(box)
#cv2.drawContours(im,[box],0,(0,0,255),2)

#fit line
rows,cols = im.shape[:2]
[vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
lefty = int((-x*vy/vx) + y)
righty = int(((cols-x)*vy/vx)+y)
cv2.line(im,(cols-1,righty),(0,lefty),(0,255,0),2)

#mostrar imagem
cv2.imshow("image",im)
cv2.waitKey(0)

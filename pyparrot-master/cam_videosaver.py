import numpy as np
import cv2 as cv

url1 = "http://192.168.0.81/img/video.mjpeg"
url2 = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"
url3 = "http://161.24.19.89/mjpg/video.mjpg"

# url1 = "http://192.168.1.105:8080/video" #eug
# url1 = "http://192.168.1.114:8080/video" #dan
# url3 = "http://192.168.1.106:8080/video" #me

cap = cv.VideoCapture(url1)
cap2 = cv.VideoCapture(url3)
cap3 = cv.VideoCapture(url2)

# Define the codec and create VideoWriter object
fourcc = cv.VideoWriter_fourcc(*'DIVX')
out = cv.VideoWriter('output.avi',fourcc, 25.0, (640,480))
out2 = cv.VideoWriter('output2.avi',fourcc, 25.0, (640,480))
out3 = cv.VideoWriter('output3.avi',fourcc, 25.0, (640,480))
i=0
print("recording...")
try:
	while(1):
		ret, frame = cap.read()
		out.write(frame)
		ret, frame = cap2.read()
		out2.write(frame)
		ret, frame = cap3.read()
		out3.write(frame)
		# if(cv.waitKey(1)!=-1):
			# break
		# i=i+1
except: KeyboardInterrupt
# Release everything if job is finished
print("done")
cap.release()
cap2.release()
cap3.release()
out.release()
out2.release()
out3.release()
cv.destroyAllWindows()
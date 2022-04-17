import numpy as np
import cv2 as cv
import glob
import os.path

url = "http://192.168.0.81/img/snapshot.cgi"
#url = "http://admin:camadmin@192.168.0.82/img/snapshot.cgi"
#url = "http://161.24.19.89/mjpg/video.mjpg"

# image = cv2.imread(filename) 

# small = cv2.resize(image, (0,0), fx=0.5, fy=0.5) 

# large = cv2.resize(image, (0,0), fx=1.5, fy=1.5)

# cv2.imshow("small image",small)
# cv2.imshow("large image",large)


#save images for calibration
calib = 0
i=1
if(calib):
	while(i<40):
		img = cv.VideoCapture(url)
		ret, img = img.read()
		name = "calib\calib%d.jpg"%i
		while(os.path.isfile(name)):
			i=i+1
			name = "calib\calib%d.jpg"%i
		cv.imwrite('calib\calib%d.jpg'%i,img)
		print(i)
		cv.waitKey(1000)
else:

	x=6
	y=10

	# termination criteria
	criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	square_size = 0.0465*1000
	objp = np.zeros((x*y,3), np.float32)
	objp[:,:2] = np.mgrid[0:y,0:x].T.reshape(-1,2)*square_size

	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.

	images = glob.glob('calib\*.jpg')
	
	for fname in images:
		cv.destroyAllWindows()
		img = cv.imread(fname)
		
		# pts4 = np.float32([[0,16],[640,16],[640,480],[0,480]]) #cam3
		# pts3 = np.float32([[0,0],[640,0],[640,480],[0,480]]) #output desejado
		# M3 = cv.getPerspectiveTransform(pts4,pts3)
		# img = cv.warpPerspective(img,M3,(640,480))
		
		#img = img[20:460,20:620]
		#img = cv.resize(img,None,fx=2, fy=2, interpolation = cv.INTER_CUBIC)
		gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
		#cv.imshow('img', img)
		#cv.waitKey(100)
		
		# Find the chess board corners
		ret, corners = cv.findChessboardCorners(gray, (y,x), None)
		
		# If found, add object points, image points (after refining them)
		if ret == True:
			objpoints.append(objp)
			corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners)
			
			# Draw and display the corners
			cv.drawChessboardCorners(img, (y,x), corners2, ret)
			#img = img[20:460,20:620]
			#img = cv.resize(img,None,fx=2, fy=2, interpolation = cv.INTER_CUBIC)
			#cv.imshow(fname, img)
			#cv.waitKey(1000)
			
	# cv.destroyAllWindows()

	ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
	print(mtx)
	print(dist)
	print("")
	
	img = cv.imread('calib\calib2.jpg')
	#cv.imshow('orig', img)
	#cv.waitKey(3000)
	h,  w = img.shape[:2]
	newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
	print(newcameramtx)
	print(roi)
	print("")
	
	# undistort
	dst = cv.undistort(img, mtx, dist, None, newcameramtx)
	#dst = cv.undistort(img, mtx, dist)
	
	# crop the image
	x, y, w, h = roi
	#dst = dst[y:y+h, x:x+w]
	
	cv.imshow('calibresult.png', dst)
	cv.imshow('orig', img)
	cv.waitKey(20000)
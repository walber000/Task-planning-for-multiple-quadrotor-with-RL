import sys
import cv2 as cv
import numpy as np
from imutils.video import WebcamVideoStream
from matplotlib import pyplot as plt

def threshold():
	img = cv.imread('img1.png',0)
	# global thresholding
	ret1,th1 = cv.threshold(img,127,255,cv.THRESH_BINARY)
	# Otsu's thresholding
	ret2,th2 = cv.threshold(img,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	# Otsu's thresholding after Gaussian filtering
	blur = cv.GaussianBlur(img,(5,5),0)
	ret3,th3 = cv.threshold(blur,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
	# plot all the images and their histograms
	images = [img, 0, th1,
			  img, 0, th2,
			  blur, 0, th3]
	titles = ['Original Noisy Image','Histogram','Global Thresholding (v=127)',
			  'Original Noisy Image','Histogram',"Otsu's Thresholding",
			  'Gaussian filtered Image','Histogram',"Otsu's Thresholding"]
	for i in range(3):
		plt.subplot(3,3,i*3+1),plt.imshow(images[i*3],'gray')
		plt.title(titles[i*3]), plt.xticks([]), plt.yticks([])
		plt.subplot(3,3,i*3+2),plt.hist(images[i*3].ravel(),256)
		plt.title(titles[i*3+1]), plt.xticks([]), plt.yticks([])
		plt.subplot(3,3,i*3+3),plt.imshow(images[i*3+2],'gray')
		plt.title(titles[i*3+2]), plt.xticks([]), plt.yticks([])
	plt.show()
	
def filter():
	#  Global Variables
	DELAY_CAPTION = 1500
	DELAY_BLUR = 100
	MAX_KERNEL_LENGTH = 31
	src = None
	dst = None
	window_name = 'Smoothing Demo'
	def main(argv):
		cv.namedWindow(window_name, cv.WINDOW_AUTOSIZE)
		# Load the source image
		imageName = argv[0] if len(argv) > 0 else 'lena.jpg'
		global src
		src = cv.imread(cv.samples.findFile(imageName))
		if src is None:
			print ('Error opening image')
			print ('Usage: smoothing.py [image_name -- default ../data/lena.jpg] \n')
			return -1
		if display_caption('Original Image') != 0:
			return 0
		global dst
		dst = np.copy(src)
		if display_dst(DELAY_CAPTION) != 0:
			return 0
		# Applying Homogeneous blur
		if display_caption('Homogeneous Blur') != 0:
			return 0
		
		for i in range(1, MAX_KERNEL_LENGTH, 2):
			dst = cv.blur(src, (i, i))
			if display_dst(DELAY_BLUR) != 0:
				return 0
		
		# Applying Gaussian blur
		if display_caption('Gaussian Blur') != 0:
			return 0
		
		for i in range(1, MAX_KERNEL_LENGTH, 2):
			dst = cv.GaussianBlur(src, (i, i), 0)
			if display_dst(DELAY_BLUR) != 0:
				return 0
		
		# Applying Median blur
		if display_caption('Median Blur') != 0:
			return 0
		
		for i in range(1, MAX_KERNEL_LENGTH, 2):
			dst = cv.medianBlur(src, i)
			if display_dst(DELAY_BLUR) != 0:
				return 0
		
		# Applying Bilateral Filter
		if display_caption('Bilateral Blur') != 0:
			return 0
		
		for i in range(1, MAX_KERNEL_LENGTH, 2):
			dst = cv.bilateralFilter(src, i, i * 2, i / 2)
			if display_dst(DELAY_BLUR) != 0:
				return 0
		
		#  Done
		display_caption('Done!')
		return 0
	def display_caption(caption):
		global dst
		dst = np.zeros(src.shape, src.dtype)
		rows, cols, _ch = src.shape
		cv.putText(dst, caption,
					(int(cols / 4), int(rows / 2)),
					cv.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255))
		return display_dst(DELAY_CAPTION)
	def display_dst(delay):
		cv.imshow(window_name, dst)
		c = cv.waitKey(delay)
		if c >= 0 : return -1
		return 0
	if __name__ == "__main__":
		main(sys.argv[1:])

url1 = "http://192.168.0.81/img/video.mjpeg"
url1 = "http://admin:camadmin@192.168.0.82/img/video.mjpeg"
stream1 = WebcamVideoStream(src=url1).start()
#stream2 = WebcamVideoStream(src=url2).start()

while(1):
	img = stream1.read()
	#img2 = stream2.read()

	# img = img[16:480,0:640]
	# img2 = img2[16:480,0:640]

	i=25
	img3 = cv.bilateralFilter(img, i, i * 2, i / 2)

	cv.imshow("imagem 1",img)
	cv.imshow("imagem 2",img3)

	if(cv.waitKey(500)!=-1):
		break
import picamera
import io
import cv2
import numpy as np
from time import sleep
from matplotlib import pyplot as plt
from matplotlib import animation as animation

def capture_contours():
	global img, clist_sample
	#Create a memory stream so photos doesn't need to be saved in a file
	stream = io.BytesIO()
	
	#Get the picture (low resolution, so it should be quite fast)
	#Here you can also specify other parameters (e.g.:rotate the image)
	with picamera.PiCamera() as camera:
	    camera.resolution = (320, 240)
	    camera.capture(stream, format='jpeg')
	
	#Convert the picture into a numpy array
	buff = np.fromstring(stream.getvalue(), dtype=np.uint8)
	
	#Now creates an OpenCV image
	img = cv2.imdecode(buff, 1)
	img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

	blur = cv2.GaussianBlur(img, (5, 5), 0)
	'''ret, img = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)'''
	thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
	clist, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	for (i, contour) in enumerate(clist):
		if cv2.matchShapes(contour, clist_sample[4], 1, 0) < 0.01:
			cv2.drawContours(img, clist, i, (0, 255, 0), 3)

''' find comparison contour from sample logo image '''
img = cv2.imread('/home/pi/Documents/picam/logo2.jpg', 0)
blur = cv2.GaussianBlur(img, (5, 5), 0)
ret, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
clist_sample, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

capture_contours()
fig = plt.figure()
im_contours = plt.imshow(img, cmap='gray', animated=True)
plt.title('Contour Image'), plt.xticks([]), plt.yticks([])

def update_images(*args):
	global img
	capture_contours()
	im_contours.set_array(img)

ani = animation.FuncAnimation(fig, update_images, blit=False)
plt.show()
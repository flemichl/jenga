import picamera
import io
import cv2
import numpy as np
from time import sleep
from matplotlib import pyplot as plt
from matplotlib import animation as animation

def capture_edges():
	global img, edges
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
	img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	edges = cv2.Canny(img,50,200)

	minLineLength = 50
	maxLineGap = 20
	lines = cv2.HoughLinesP(edges,1,np.pi/180,10,minLineLength,maxLineGap)
	if lines != None:
		for x1,y1,x2,y2 in lines[0]:
    			cv2.line(img,(x1,y1),(x2,y2),(255,0,0),2)

capture_edges()
fig = plt.figure()
fig.add_subplot(121)
im_orig = plt.imshow(img, cmap='gray', animated=True)
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
fig.add_subplot(122)
im_edges = plt.imshow(edges, cmap='gray', animated=True)
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

def update_images(*args):
	global img, edges
	capture_edges()
	im_orig.set_array(img)
	im_edges.set_array(edges)

ani = animation.FuncAnimation(fig, update_images, blit=False)
plt.show()
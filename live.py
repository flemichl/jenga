#!/usr/bin/env python

import picamera
import io
import cv2
import numpy as np
from time import sleep
from matplotlib import pyplot as plt
from matplotlib import animation as animation

camera = picamera.PiCamera()
camera.resolution = (320, 240)
camera.hflip = True
camera.vflip = True

def capture():
	global img
	stream = io.BytesIO()
	camera.capture(stream, format='jpeg')
	
	#Convert the picture into a numpy array
	buff = np.fromstring(stream.getvalue(), dtype=np.uint8)
	
	#Now creates an OpenCV image
	img = cv2.imdecode(buff, 1)
	img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def update_image(*args):
	capture()
	im_view.set_array(img)


fig = plt.figure()
capture()
im_view = plt.imshow(img, cmap='gray', animated=True)
plt.title('Jengrabber Cam'), plt.xticks([]), plt.yticks([])

ani = animation.FuncAnimation(fig, update_image, blit=False)
plt.show()

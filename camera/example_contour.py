import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('/home/pi/Documents/picam/logo2.jpg', 0)
blur = cv2.GaussianBlur(img, (5, 5), 0)
ret, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, 4, (0, 255, 0), 3)

print len(contours)

plt.subplot(121),plt.imshow(thresh, cmap='gray')
plt.title('Threshold'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(img, cmap='gray')
plt.title('Contours'), plt.xticks([]), plt.yticks([])

plt.show()
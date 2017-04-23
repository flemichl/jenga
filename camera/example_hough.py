import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('jenga_example.jpg',0)
edges = cv2.Canny(img,150,200,apertureSize = 3)

minLineLength = 50
maxLineGap = 10
lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
for x1,y1,x2,y2 in lines[0]:
    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

cv2.imwrite('houghlines.jpg',img)

plt.subplot(121),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(img,cmap = 'gray')
plt.title('Lines Detected'), plt.xticks([]), plt.yticks([])

plt.show()
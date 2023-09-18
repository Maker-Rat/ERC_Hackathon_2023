import cv2
import numpy as np

light_red = np.array([0,50,50])
dark_red = np.array([10,255,255])

img = cv2.imread("/home/ritwik/Pictures/cone.png")
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(img_hsv, light_red, dark_red)
cv2.imshow("pic", mask)
cv2.waitKey(0)
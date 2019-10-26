#!/usr/bin/env python

import numpy as np
import cv2
import imutils 

image = cv2.imread("Images/tennis.jpg")
cv2.imshow("Original", image)

#convert image to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
cv2.imshow("HSV Image", hsv)

#find lower and upper bounds of yellow color
yellowLower = (30, 100, 100)
yellowUpper = (60, 255, 255)

#define mask using lower and upper bounds of yellow color
mask = cv2.inRange(hsv, yellowLower, yellowUpper)

cv2.imshow("Mask", mask)



cv2.waitKey(0)
cv2.destroyAllWindows()
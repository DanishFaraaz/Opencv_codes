#!/usr/bin/env python

import numpy as np

import cv2

image_name =  'flower'

print 'Read an image from file'
color_image = cv2.imread("Images/flower.jpeg", cv2.IMREAD_COLOR)

print 'Display image in native color'
cv2.imshow("Original_image", color_image)
cv2.moveWindow("Original Image", 0, 0)
print(color_image.shape)

height, width, channel = color_image.shape

print 'Split the image into 3 channels'
blue, green, red = cv2.split(color_image)

cv2.imshow("Blue channel", blue)
cv2.moveWindow("Blue Channel", 0, height)

cv2.imshow("Red channel", red)
cv2.moveWindow("Red Channel", 0, height)

cv2.imshow("Green channel", green)
cv2.moveWindow("Green Channel", 0, height)

print('----Split image into Hue, Saturation and Value channels----')
hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)
hsv_image = np.concatenate((h, s, v), axis=1)
cv2.imshow("Hue, Saturation and Value image", hsv_image)

print '----Convert image to grayscale----'
gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
cv2.imshow("Gray Image", gray_image)

cv2.waitKey(0)
cv2.destroyAllWindows
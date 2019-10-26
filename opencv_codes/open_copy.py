#!/usr/bin/env python

import numpy as np

import cv2

image_name = "flower"

print 'read an image from file'
img = cv2.imread("Images/flower.jpeg")

print 'create a  window to display the image'
cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

print 'display the image'
cv2.imshow("Image", img)

print 'press a key in the keyboard to make a copy'
cv2.waitKey(0)

print 'Image copied to folder Images/copy'
cv2.imwrite("Images/Copy/flower-copy.jpeg", img)
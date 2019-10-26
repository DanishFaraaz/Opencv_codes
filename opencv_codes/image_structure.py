#!/usr/bin/env python


import numpy as np

import cv2

print 'Read an image from the file'
img = cv2.imread("Images/blackwhite.jpg")

print 'Displaying the content of the image'
print img

print 'Type of the image is : %s'%type(img)
print 'Size of the image is : %d'%img.size
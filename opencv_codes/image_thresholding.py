#!/usr/bin/env python


import numpy as np
import cv2

def read_image(image_name, as_gray):
    if as_gray:
        image = cv2.imread(image_name, cv2.IMREAD_GRAYSCALE)
    else:
        image = cv2.imread(image_name, cv2.IMREAD_COLOR)
    cv2.imshow("RGB Image", image)
    return image

def basic_thresholding(gray_image, threshold_value):
    ret, thresh_basic = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow("Basic Binary Image", thresh_basic)

def adaptive_thresholding(rgb_image, threshold_value):
    adaptive_threshold_image = cv2.adaptiveThreshold(rgb_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, threshold_value, 2)
    cv2.imshow("Adaptive Threshold Image", adaptive_threshold_image)    

def main():
    image_name = "Images/flower.jpeg"
    as_gray = True
    threshold_value = 115
    gray_image = read_image(image_name, as_gray)
    basic_thresholding(gray_image, threshold_value)
    adaptive_thresholding(gray_image, threshold_value)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
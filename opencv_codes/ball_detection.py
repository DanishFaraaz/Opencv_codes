#!/usr/bin/env python

import numpy as np
import cv2

def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show:
        cv2.imshow("RGB Image", rgb_image)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #converting image to HSV format
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV Image", hsv)

    #defining a mask using lower and upper bounds
    mask  = cv2.inRange(hsv, lower_bound_color, upper_bound_color)

    return mask

def getContours(binary_image):
    contours, hierarchy = cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours

def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1], 3], 'uint8')

    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if(area>100):
            cv2.drawContours(rgb_image, [c], -1, (150, 250, 150), 1)
            cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
            cv2.circle(black_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
            print("Area: {}, Perimeter: {}".format(area, perimeter))
    print("Number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image contours", rgb_image)
    cv2.imshow("Black Image contours", black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00']!=0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy

def main():
    image_name = "Images/tennis03.jpg"
    
    #upper and lower bounds of yellow color
    yellowLower = (30, 50, 100)
    yellowUpper = (50, 355, 255)

    rgb_image = read_rgb_image(image_name, True)
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    draw_ball_contour(binary_image_mask, rgb_image, contours)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()




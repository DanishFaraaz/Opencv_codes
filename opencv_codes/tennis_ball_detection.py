#!/usr/bin/env python

import numpy as np
import cv2

#path to the video file
video_capture = cv2.VideoCapture('Video/tennisball.mp4')

def main():
    while (True):
        #Reading the video
        ret, frame = video_capture.read()

        #Converting to HSV format
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        #Defining lower and upper bound colors
        lower_bound_color = (30, 50, 100)
        upper_bound_color = (35, 255, 255)

        #Defining the mask
        mask = cv2.inRange(hsv, lower_bound_color, upper_bound_color)

        #Getting the required contours
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #Drawing the contours
        black_image = np.zeros([mask.shape[0], mask.shape[1], 3], 'uint8')

        for c in contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if(area>100):
                cv2.drawContours(frame, [c], -1, (150, 250, 150), 1)
                cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
                cx, cy = get_contour_center(c)
                cv2.circle(frame, (cx, cy), (int)(radius), (0, 0, 255), 1)
                cv2.circle(black_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
                print("Area: {}, Perimeter: {}".format(area, perimeter))
        print("Number of contours: {}".format(len(contours)))
        cv2.imshow("RGB Image contours", frame)
        cv2.imshow("Black Image contours", black_image)


        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00']!=0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy

if __name__=='__main__':
    main()


video_capture.release()
cv2.destroyAllWindows
#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def callback(ros_image):

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "brg8")
    except CvBridgeError as e:
        print(e)

    
    while (True):
        #Converting to HSV format
        hsv = cv2.cvtColor(ros_image, cv2.COLOR_BGR2HSV)
            
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
                cv2.drawContours(ros_image, [c], -1, (150, 250, 150), 1)
                cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
                cx, cy = get_contour_center(c)
                cv2.circle(ros_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
                cv2.circle(black_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
                print("Area: {}, Perimeter: {}".format(area, perimeter))
        print("Number of contours: {}".format(len(contours)))
        cv2.imshow("RGB Image contours", ros_image)
        cv2.imshow("Black Image contours", black_image)

        if cv2.waitKey(10) & 0xFF==ord('q'):
            break

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00']!=0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy



def main():
    rospy.init_node("Tennis_Ball_Listener", anonymous=True)

    sub = rospy.Subscriber("tennis_ball_image", Image, callback)

    rospy.spin()

if __name__=='__main__':
    main()
#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image

video_capture = cv2.VideoCapture(0)

def main():
    rospy.init_node('Tennis Ball Publisher', anonymous = True)

    pub = rospy.Publisher("tennis_ball_image", Image, queue_size = 10)

    rate = rospy.Rate(1)

    while (True):
        ret, frame = video_capture.read()

        pub.publish(frame)

        rate.sleep()
        

if __name__=='__main__':
    main()

#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower():

    def __init__(self):
         self.start = True

    def track_line(self, flipped_img):
        # We get image dimensions and crop the parts of the image we dont need

        img = cv2.flip(flipped_img,-1)
        height, width, channels = img.shape
        crop_img = img[int((height/2)+130):int((height/2)+160)][1:int(width)]
        start_point = (0,int(height/2)+130)
        end_point = (int(width),int(height/2)+160)
        color = (255,0,0)
        cv2.rectangle(img,start_point,end_point,color,3)
        cv2.imshow("Cropped box",img)

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV, run the intrinsic calibration from Autorace package and tune the HSV low and high values manually
        """

        # Threshold the HSV image to get only yellow colors

        lower_yellow = np.array([9,15,40])	
        upper_yellow = np.array([42,220,197])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = (m['m10']/m['m00']), (m['m01']/m['m00'])
            counter = 1
        except ZeroDivisionError:
            cx, cy = height/2, width/2
            counter = 0
        
        # Draw the centroid in the resultut image
        #cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 5,(0,0,255),-1)
        cv2.imshow("Original", hsv)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)
        
        #controller to keep in lane
        twist_object = Twist()
        
        #calculating the error term
        twist_object.linear.x = 0.02
        
        #distance of the centroid of the blob w.r.t image centre
        error = (width/2 - cx)

        if counter == 1:
               twist_object.angular.z = 0.0008*error
        else:
               twist_object.linear.x = 0.0
               twist_object.angular.z = 0.1
        return twist_object, error
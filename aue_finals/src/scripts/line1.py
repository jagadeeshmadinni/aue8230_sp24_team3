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
        crop_img1 = img[int((height/2)+120):int((height/2)+160)][1:int(width)]
        start_point1 = (0,int(height/2)+120)
        end_point1 = (int(width),int(height/2)+160)
        colorb = (255,0,0)
        cv2.rectangle(img,start_point1,end_point1,colorb,3)
        cv2.imshow("Cropped box",img)

        crop_img2 = img[int((height/2)+80):int((height/2)+120)][1:int(width)]
        start_point2 = (0,int(height/2)+80)
        end_point2 = (int(width),int(height/2)+120)
        colorg = (0,255,0)
        cv2.rectangle(img,start_point2,end_point2,colorg,3)
        cv2.imshow("Cropped box",img)

        # Convert from RGB to HSV
        hsv1 = cv2.cvtColor(crop_img1, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(crop_img2, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV, run the intrinsic calibration from Autorace package and tune the HSV low and high values manually
        """

        # Threshold the HSV image to get only yellow colors

        lower_yellow = np.array([90,0,0])	
        upper_yellow = np.array([140,22,255])
        mask1 = cv2.inRange(hsv1, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(hsv2, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m1 = cv2.moments(mask1, False)
        m2 = cv2.moments(mask2, False)

        try:
            cx1, cy1 = (m1['m10']/m1['m00']), (m1['m01']/m1['m00'])
            counter_slit1 = 1
        except ZeroDivisionError:
            cx1, cy1 = height/2, width/2
            counter_slit1 = 0
        
        try:
            cx2, cy2 = (m2['m10']/m2['m00']), (m2['m01']/m2['m00'])
            counter_slit2 = 1
        except ZeroDivisionError:
            cx2, cy2 = height/2, width/2
            counter_slit2 = 0

        # Draw the centroid in the resultut image
        #cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask1,(int(cx1), int(cy1)), 5,(0,0,255),-1)
        cv2.circle(mask2,(int(cx2), int(cy2)), 5,(0,0,255),-1)
        cv2.imshow("Original", hsv1)
        cv2.imshow("Original", hsv2)
        cv2.imshow("MASK", mask1)
        cv2.imshow("MASK", mask2)
        cv2.waitKey(1)
        
        #controller to keep in lane
        twist_object = Twist()
        
        #calculating the error term
        twist_object.linear.x = 0.05
        
        #distance of the centroid of the blob w.r.t image centre
        error1 = (width/2 - cx1)
        error2 = (width/2 - cx2)

        if counter_slit1 == 1 and counter_slit2 == 1:
               twist_object.angular.z = 0.0008*error1 + 0.0004*error2
        elif counter_slit1 == 1 and not(counter_slit2 == 0):
               twist_object.angular.z = 0.0008*error1
        elif counter_slit1 == 0 and not(counter_slit2 == 1):
               twist_object.angular.z = 0.0004*error2
        else:
               twist_object.linear.x = 0.0
               twist_object.angular.z = 0.1
        return twist_object, error1
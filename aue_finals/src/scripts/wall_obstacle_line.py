#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np

class WallFollower:
        
    def __init__(self):
        self.move_wall = Twist()            

    def follow_wall(self,data):
        #lidar_scan = list(data.ranges[0:359])
        scan = [x for x in data if x < 3]     # Filtering garbage values
        low = 10
        scan = [low if element == 0 else element for element in scan]
        #print("scan is %s" % scan)

        #right = sum(scan[-90:-16])/len(scan[-90:-16])   # Average range
        #left = sum(scan[16:90])/len(scan[16:90])        # Average range
        ##front = sum(scan[0:15]+scan[-1:-15])/len(scan[0:15]+scan[-1:-15])   # Average range
        #front = min(scan[0:10]+scan[-1:-10])
        #Lf = sum(scan[13:18])/len(scan[13:18])          # Average range
        #Rf = sum(scan[-18:-13])/len(scan[-18:-13])      # Average range

        #Copied lines
        right1 = sum(scan[-90:-75])/len(scan[-90:-75])   # Average range
        right2 = sum(scan[-74:-40])/len(scan[-74:-40])   # Average range
        right3 = sum(scan[-39:-16])/len(scan[-39:-16])   # Average range
        right = min(right1,right2,right3)

        left1 = sum(scan[75:90])/len(scan[75:90])   # Average range
        left2 = sum(scan[40:74])/len(scan[40:74])   # Average range
        left3 = sum(scan[16:39])/len(scan[16:39])   # Average range
        left = min(left1,left2,left3)

        front = min(scan[0:20]+scan[-1:-20])
        front_stop_sign = min(scan[0:30]+scan[-1:-30])
        Lf = min(scan[13:18])          # Average range
        Rf = min(scan[-18:-13])      # Average range

        print("Front range is %s" % front)

        linear_vel = 0.15
        angular_vel = 0
        front_threshold = 0.5
        obst_threshold = 0.1
        
        error = left - right

        obs_dist = abs(Lf-Rf)

        self.move_wall.linear.x = linear_vel
        self.move_wall.angular.z = angular_vel + 0.9*error          # Yaw P control
        #print("Angular Velocity is %s" % self.move_wall.angular.z)

        #if front<front_threshold:
        if front<front_threshold or obs_dist<obst_threshold:
            self.move_wall.linear.x = 0
            #print("Bot is near obstacle")

        return self.move_wall, front, front_stop_sign
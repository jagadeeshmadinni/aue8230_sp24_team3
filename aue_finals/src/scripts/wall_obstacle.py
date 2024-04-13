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
<<<<<<< HEAD
        low = 10
        res = [low if element == 0 else element for element in scan]
=======

>>>>>>> 2bb1907b1f07648ae308df6c2e73cb4bacf489d9
        print("scan is %s" % scan)

        right = sum(scan[-90:-16])/len(scan[-90:-16])   # Average range
        left = sum(scan[16:90])/len(scan[16:90])        # Average range
        #front = sum(scan[0:15]+scan[-1:-15])/len(scan[0:15]+scan[-1:-15])   # Average range
<<<<<<< HEAD
        front = min(scan[0:10]+scan[-1:-10])
        Lf = sum(scan[13:18])/len(scan[13:18])          # Average range
        Rf = sum(scan[-18:-13])/len(scan[-18:-13])      # Average range

        print("Front range is %s" % front)

=======
        front = min(scan[0:15]+scan[-1:-15])
        Lf = sum(scan[13:18])/len(scan[13:18])          # Average range
        Rf = sum(scan[-18:-13])/len(scan[-18:-13])      # Average range
        print("Front range is %s" % front)
>>>>>>> 2bb1907b1f07648ae308df6c2e73cb4bacf489d9
        linear_vel = 0.15
        angular_vel = 0
        front_threshold = 0.4
        obst_threshold = 0.1
        
        error = left - right

<<<<<<< HEAD
        obs_dist = abs(Lf-Rf)

=======
>>>>>>> 2bb1907b1f07648ae308df6c2e73cb4bacf489d9
        self.move_wall.linear.x = linear_vel
        self.move_wall.angular.z = angular_vel + 0.9*error          # Yaw P control
        print("Angular Velocity is %s" % self.move_wall.angular.z)

        #if front<front_threshold:
        if front<front_threshold or obs_dist<obst_threshold:
            self.move_wall.linear.x = 0
            print("Bot is near obstacle")
        return self.move_wall
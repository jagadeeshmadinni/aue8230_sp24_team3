#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower:
    def __init__(self):
        #rospy.init_node('wall_follower')
        self.move = Twist()

    def follow_wall(self,begin_flag):

        if(begin_flag==True):
            self.move.linear.x = 1
            self.move.linear.y = 1
            self.move.linear.z = 0
            self.move.angular.x = 0
            self.move.angular.y = 0
            self.move.angular.z = 1

        else:
            self.move.linear.x = 0
            self.move.linear.y = 0
            self.move.linear.z = 0
            self.move.angular.x = 0
            self.move.angular.y = 0
            self.move.angular.z = 0
        return self.move

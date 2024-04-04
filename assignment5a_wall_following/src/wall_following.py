#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.move = Twist()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.wallfollow)
        rospy.on_shutdown(self.myhook)
    
    def PID(self, error, error_prev, Kp=0.9, Kd=1):
      
        return Kp * error + Kd*(error-error_prev)
    
    def wallfollow(self, data):
        lidar_scan = list(data.ranges[0:359])
        scan = [x for x in lidar_scan if x < 3]     # Filtering garbage values
        #scan = lidar_scan

        right = sum(scan[-90:-16])/len(scan[-90:-16])   # Average range
        left = sum(scan[16:90])/len(scan[16:90])        # Average range
        front = sum(scan[0:15]+scan[-1:-15])/len(scan[0:15]+scan[-1:-15])   # Average range
        Lf = sum(scan[13:18])/len(scan[13:18])          # Average range
        Rf = sum(scan[-18:-13])/len(scan[-18:-13])      # Average range

        linear_vel = 0.15
        angular_vel = 0
        front_threshold = 0.5
        obst_threshold = 0.3
        
        error = left - right
        error_prev = error

        obs_dist = abs(Lf-Rf)

        self.move.linear.x = linear_vel
        self.move.angular.z = angular_vel + self.PID(error, error_prev)     # Yaw PD control
        print("Angular Velocity is %s" % self.move.angular.z)

        if front<front_threshold and obs_dist<obst_threshold:
            self.move.linear.x = 0
            print("Bot is near obstacle")
        #else:
            #self.move.linear.x = linear_vel

    # https://get-help.theconstruct.ai/t/how-to-stop-your-robot-when-ros-is-shutting-down/225
    def myhook(self):    
        print("shutdown time!")
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pub.publish(self.move)
        
    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.move)
    
if __name__ == "__main__":
    wall_follower = WallFollower()
    wall_follower.run()
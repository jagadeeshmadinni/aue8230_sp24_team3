#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi
import numpy as np
import time

def callback(msg):
    dists = np.array([msg.ranges[0],msg.ranges[30],msg.ranges[90],msg.ranges[270],msg.ranges[330]])    
    dist = np.where(dists > 3.5, 3.5, dists)
    J = [i * i * 100 for i in dist]
    opt = np.argmax(J)
    optopt = np.min(opt)
    print ('Direction Chosen   {}'.format(optopt))
    U = np.array([[0.2, 0.2, 0, 0, 0.2],[0, -1, -1, 1, 1]])
    vel_msg.linear.x = U[0][optopt]
    vel_msg.angular.z = U[1][optopt]
    vel_pub.publish(vel_msg)

rospy.init_node('obstacle_avoidance', anonymous=True)

vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages

lidar_sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages

vel_msg = Twist()

vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
        
delay = 3
time.sleep(delay)
rospy.spin() # Infinite loop until someone stops the program

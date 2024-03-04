#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

def move():
    # Starts a new node
    rospy.init_node('circle', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
 
    vel_msg.linear.x = 0.3
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0.3

    while not rospy.is_shutdown():

        velocity_publisher.publish(vel_msg)
        #After the loop, stops the robot
        #vel_msg.linear.x = 0
        #vel_msg.angular.z = 0
        #Force the robot to stop
        #velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass

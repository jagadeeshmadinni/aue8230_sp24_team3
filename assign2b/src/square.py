#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi
#pi = 3.1415926535
   
def move():
    rospy.init_node('turtlebot_controller', anonymous=True)

    velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

    vel_msg = Twist()
    
    distance = 2
    forwardvel = 0.3
    angvel = 0.3
    
    vel_msg.linear.x = forwardvel 
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    no_of_sides = 0

    while not rospy.is_shutdown():
    
        while(no_of_sides < 4):
            no_of_sides=no_of_sides+1
            current_distance = 0
        
            t0 = rospy.Time.now().to_sec()
        
            while(current_distance < distance):
                velocity_publisher.publish(vel_msg)
                t1=rospy.Time.now().to_sec()
                current_distance= forwardvel*(t1-t0)
            
            vel_msg.linear.x = 0
            vel_msg.angular.z = angvel
            t0 = rospy.Time.now().to_sec()
            current_heading = 0
            while(current_heading < (90*pi/180)):
                velocity_publisher.publish(vel_msg)
                t1=rospy.Time.now().to_sec()
                current_heading= angvel*(t1-t0)
                
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0  
            velocity_publisher.publish(vel_msg)    
            
            vel_msg.linear.x = forwardvel
            vel_msg.angular.z = 0   
            
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0  
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass

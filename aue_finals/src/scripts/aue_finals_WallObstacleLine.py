#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
import numpy as np
from wall_obstacle_line import WallFollower
from line import LineFollower
import imagezmq
import cv2
from stop_sign import StopSign
class Autonomy_Final:

    def __init__(self):
        rospy.init_node('i_am_groot', anonymous=True)
        self.velocity = Twist()
        zeros = [0]*360
        self.lidar_data = zeros
        self.Velocity_Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.Range_Subscriber = rospy.Subscriber("/scan", LaserScan, self.range_callback)
        self.stop_sign_pub = rospy.Publisher('/stop_sign', Bool, queue_size=10)
        self.bbox_pub = rospy.Publisher('/bbox',Int32MultiArray,queue_size=10)
        #self.Image_Subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.Line_Following_object.camera_callback)
        rospy.on_shutdown(self.Shutdown_callback)
        self.last_cmdvel_command = Twist()
        #self._cmdvel_pub_rate = rospy.Rate(1)

    def Shutdown_callback(self):
        print("Shutting down now")
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.Velocity_Publisher.publish(self.velocity)
        ctrl_c = True

    def cmdvel_callback(self,msg):
        self.last_cmdvel_command = msg
    
    def range_callback(self,msg):
        self.lidar_data = list(msg.ranges[0:359])
         
    def move_robot(self, twist_object):
        self.Velocity_Publisher.publish(twist_object)
        #self._cmdvel_pub_rate.sleep()
            #current_equal_to_new = self.compare_twist_commands(twist1=self.last_cmdvel_command,twist2=twist_object)
    
    '''def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)'''


    def motion_planner(self):
        # Start the supervisory node
        #self.Autonomy = Autonomy_Final()
        #lidar_scan = list(data_lidar.ranges[0:359])
        endMission = False
        switch_flag = True
        previous = 0
        stop_sign_flag = False
        stop_sign_threshold = 0.5
        image_hub = imagezmq.ImageHub()
        Wall_Following_object = WallFollower()
        Line_Following_object = LineFollower()
        Stop_Sign_Object = StopSign()
        print("Mission commenced")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():

            # Check for obstacles and line

            rpi_name, cv_image = image_hub.recv_image()
            cv2.waitKey(1)
            image_hub.send_reply(b'OK')
            #cv2.imshow(rpi_name, cv_image)
            cmd_vel_wall, front = Wall_Following_object.follow_wall(data=self.lidar_data)   
            cmd_vel_line, error_line = Line_Following_object.track_line(flipped_img=cv_image)
            stop_sign_flag,bbox_coords = Stop_Sign_Object.stop_sign_detect(cv_image)
            self.stop_sign_pub.publish(stop_sign_flag)
            self.bbox_pub.publish(Int32MultiArray(data=bbox_coords))

            # Velocity commands
            #self.velocity.angular.z = cmd_vel_wall.angular.z
            #self.velocity.linear.x = cmd_vel_wall.linear.x

            # Check for the error published by line following algorithm
            #error_diff =  np.append(previous,error_line)
            #if np.diff(error_diff) != 0:
            self.velocity.linear.x = cmd_vel_line.linear.x
            self.velocity.angular.z = cmd_vel_line.angular.z
            # Check for stop sign
            '''if stop_sign_flag == True and front < stop_sign_threshold:
                self.velocity.linear = 0
                self.velocity.angular.z = 0
                rospy.sleep(3)'''

            self.move_robot(self.velocity)
            #previous = error_line
            #rate.sleep()

if __name__ == "__main__":
    jarvis = Autonomy_Final()
    jarvis.motion_planner()
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
from wall_obstacle_line_gazebo import WallFollower
from line_gazebo import LineFollower
import imagezmq
import cv2
from stop_sign import StopSign
import sys,tty,os,termios
class Autonomy_Final:

    def __init__(self):
        rospy.init_node('i_am_groot', anonymous=True)
        self.bridge_object = CvBridge()
        self.velocity = Twist()
        zeros = [0]*360
        self.lidar_data = zeros
        self.cv_image = None
        self.switch_flag = False
        self.Velocity_Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.Range_Subscriber = rospy.Subscriber("/scan", LaserScan, self.range_callback)
        self.stop_sign_pub = rospy.Publisher('/stop_sign', Bool, queue_size=10)
        self.switch_flag_sub = rospy.Subscriber('/switch_flag', Bool, self.switch_flag_callback)
        self.bbox_pub = rospy.Publisher('/bbox',Int32MultiArray,queue_size=10)
        self.Image_Subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback) #gazebo
        rospy.on_shutdown(self.Shutdown_callback)
        self.last_cmdvel_command = Twist()

    def Shutdown_callback(self):
        #print("Shutting down now")
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.Velocity_Publisher.publish(self.velocity)
        ctrl_c = True

    def camera_callback(self, data):
        #print("Camera callback is called")
        # We select bgr8 because its the opneCV encoding by default
        # print("in line_follower_callback")
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

    def cmdvel_callback(self,msg):
        self.last_cmdvel_command = msg

    def switch_flag_callback(self,msg):
        self.switch_flag = msg
    
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

        previous = 0
        previous_front = 0
        stop_sign_flag = False
        stop_sign_done = False
        stop_sign_threshold = 0.2
        Line_Blob = 0
        #image_hub = imagezmq.ImageHub()
        Wall_Following_object = WallFollower()
        Line_Following_object = LineFollower()
        Stop_Sign_Object = StopSign()
        #print("Mission commenced")
        rate = rospy.Rate(2)
        #with Listener(on_press=capture_key) as listener:
        #    listener.join()    

        while not rospy.is_shutdown():
            
            if self.cv_image is not None:
                    
                # Check for obstacles and line
                if self.switch_flag is False:
                    cmd_vel_wall, front, front_stop_sign = Wall_Following_object.follow_wall(data=self.lidar_data)   
                    cmd_vel_line, error_line = Line_Following_object.track_line(flipped_img=self.cv_image)

                    # Check for the error published by line following algorithm
                    error_diff =  np.append(previous,error_line)

                    # Check for error published by wall following algorithm
                    error_front = np.append(previous_front,front)
                    print("Wall error is %s" % error_front)
                    print("Line error is %s" % error_line)

                    # Velocity commands
                    self.velocity.angular.z = cmd_vel_wall.angular.z
                    self.velocity.linear.x = cmd_vel_wall.linear.x

                    #self.switch_flag = (np.diff(error_front) == 0 and np.diff(error_diff) != 0 and front>2)
                    previous = error_line
                    previous_front = front
                    print("Wall linear velocity is %s" % cmd_vel_wall.linear.x)
                    print("Wall angular velocity is %s" % cmd_vel_wall.angular.z)
                    print("Switch Flag is %s" % self.switch_flag)
                    print("Line error is %s" % error_diff)
                    print("Wall error is %s" % error_front)
                    print("Front distance is %s" % front)

                # Start line following
                else:
                    scan = [x for x in self.lidar_data if x < 3]     # Filtering garbage values
                    low = 10
                    scan = [low if element == 0 else element for element in scan]
                    front_stop_sign = min(scan[0:30]+scan[-1:-30])
                    cmd_vel_line, error_line = Line_Following_object.track_line(flipped_img=self.cv_image)
                    stop_sign_flag,bbox_coords = Stop_Sign_Object.stop_sign_detect(self.cv_image)
                    self.stop_sign_pub.publish(stop_sign_flag)
                    self.bbox_pub.publish(Int32MultiArray(data=bbox_coords))
                    self.velocity.linear.x = cmd_vel_line.linear.x
                    self.velocity.angular.z = cmd_vel_line.angular.z
                    print("Line linear velocity is %s" % cmd_vel_line.linear.x)
                    print("Line angular velocity is %s" % cmd_vel_line.angular.x)

                    # Check for stop sign
                    if (stop_sign_flag == True) and (front_stop_sign < stop_sign_threshold) and (stop_sign_done == False):
                        self.velocity.linear.x = 0
                        self.velocity.angular.z = 0
                        self.move_robot(self.velocity)
                        print("Stop time 0")
                        rospy.sleep(3)
                        print("Stop time 3")
                        stop_sign_done = True
                        print("Stop Sign %s" % stop_sign_flag)


                self.move_robot(self.velocity)
                print("Published linear velocity is %s" % self.velocity.linear.x)
                print("Published angular velocity is %s" % self.velocity.angular.x)

            rate.sleep()

if __name__ == "__main__":
    jarvis = Autonomy_Final()
    jarvis.motion_planner()
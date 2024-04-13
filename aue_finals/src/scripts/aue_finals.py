#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from wall_obstacle import WallFollower

class Autonomy_Final:

    def __init__(self):
        rospy.init_node('i_am_groot', anonymous=True)
        self.velocity = Twist()
        zeros = [0]*360
        self.lidar_data = zeros
        self.Velocity_Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.Range_Subscriber = rospy.Subscriber("/scan", LaserScan, self.range_callback)
        #self.Image_Subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.Line_Following_object.camera_callback)
        rospy.on_shutdown(self.Shutdown_callback)
        self.last_cmdvel_command = Twist()
        #self._cmdvel_pub_rate = rospy.Rate(3)

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
                                    
    def motion_planner(self):
        # Start the supervisory node
        #self.Autonomy = Autonomy_Final()
        #lidar_scan = list(data_lidar.ranges[0:359])
        endMission = False
        switch_flag = True
        while not rospy.is_shutdown():
            print("Mission commenced")
            while(not endMission):
                # Check for obstacles and line
                Wall_Following_object = WallFollower()
                cmd_vel = Wall_Following_object.follow_wall(data=self.lidar_data)
                self.move_robot(cmd_vel)

if __name__ == "__main__":
    jarvis = Autonomy_Final()
    jarvis.motion_planner()
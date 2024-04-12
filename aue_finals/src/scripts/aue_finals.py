#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from wall_obstacle import WallFollower

class Autonomy_Final:

    def __init__(self):
        rospy.init_node('i_am_groot', anonymous=True)
        self.velocity = Twist()
        self.Velocity_Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self.Range_Subscriber = rospy.Subscriber("/scan", LaserScan, self.Wall_Following_object.wallfollow)
        #self.Image_Subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.Line_Following_object.camera_callback)
        rospy.on_shutdown(self.Shutdown_callback)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)

    def Shutdown_callback(self):
        print("Shutting down now")
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.Velocity_Publisher.publish(self.velocity)
        ctrl_c = True

    def cmdvel_callback(self,msg):
        self.last_cmdvel_command = msg
    
    def compare_twist_commands(self,twist1,twist2):
        LX = twist1.linear.x == twist2.linear.x
        LY = twist1.linear.y == twist2.linear.y
        LZ = twist1.linear.z == twist2.linear.z
        AX = twist1.angular.x == twist2.angular.x
        AY = twist1.angular.y == twist2.angular.y
        AZ = twist1.angular.z == twist2.angular.z
        equal = LX and LY and LZ and AX and AY and AZ
        if not equal:
            rospy.logwarn("The Current Twist is not the same as the one sent, Resending")
        return equal

    def move_robot(self, twist_object):
        # We make this to avoid Topic loss, specially at the start
        current_equal_to_new = False
        while (not (current_equal_to_new) ):
            self.Velocity_Publisher.publish(twist_object)
            self._cmdvel_pub_rate.sleep()
            current_equal_to_new = self.compare_twist_commands(twist1=self.last_cmdvel_command,
                                    twist2=twist_object)
                                    
    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)


    def motion_planner(self):
        # Start the supervisory node
        self.Autonomy = Autonomy_Final()
        endMission = False
        switch_flag = True
        while not rospy.is_shutdown():
            print("Mission commenced")
            while(not endMission):
                # Check for obstacles and line
                self.Wall_Following_object = WallFollower()
                cmd_vel = self.Wall_Following_object.follow_wall(switch_flag)
                self.move_robot(cmd_vel)

if __name__ == "__main__":
    jarvis = Autonomy_Final()
    jarvis.motion_planner()
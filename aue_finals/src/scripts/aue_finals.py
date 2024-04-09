#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

class MoveTurtlebot3(object):

    def __init__(self):
    
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_subs = rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)


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
            self.cmd_vel_pub.publish(twist_object)
            self._cmdvel_pub_rate.sleep()
            current_equal_to_new = self.compare_twist_commands(twist1=self.last_cmdvel_command,
                                    twist2=twist_object)
                                    
    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)

class WallFollower:
    def __init__(self):
        #rospy.init_node('wall_follower')
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

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        #rospy.on_shutdown(self.myhook)

    def camera_callback(self, data):

        self.twist_object = Twist()

        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
        #crop_img = cv_image[340:360][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])		
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            counter = 1
        except ZeroDivisionError:
            cx, cy = height/2, width/2
            counter = 0
        
        # Draw the centroid in the resultut image
        #cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 5,(0,0,255),-1)
        cv2.imshow("Original", hsv)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)
        
        #controller to keep in lane
        #twist_object = Twist()
        
        #calculating the error term
        self.twist_object.linear.x = 0.1
        
        #distance of the centroid of the blob w.r.t image centre
        error = width/2 - cx
        if counter == 1:
               self.twist_object.angular.z = 0.0008*error
        elif counter == 0:
               self.twist_object.angular.z = -0.01

        #print(width/2,cx)
        #rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        rospy.loginfo("ERROR SENT===>"+str(error))
        #rospy.loginfo(err)
        # Make it start turning
        #self.moveTurtlebot3_object.move_robot(twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()
'''
    def myhook(self):    
        print("shutdown time!")
        self.twist_object.linear.x = 0
        self.twist_object.angular.z = 0
        self.pub.publish(self.twist_object)
'''

class Autonomy_Final:

    def __init__(self):
        rospy.init_node('Supervisory_Node', anonymous=True)
        self.velocity = Twist()
        self.Velocity_Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.Range_Subscriber = rospy.Subscriber("/scan", LaserScan, self.Wall_Following_object.wallfollow)
        self.Image_Subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.Line_Following_object.camera_callback)
        rospy.on_shutdown(self.Shutdown_callback)

    def Shutdown_callback(self):
        print("Shutting down now")
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.Velocity_Publisher.publish(self.velocity)
        ctrl_c = True
'''
    def Autonomy_Run(self):
        while not rospy.is_shutdown():
            self.Velocity_Publisher.publish(self.velocity)
'''

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(2)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

def motion_planner(self):
    # Start the supervisory node
    self.Autonomy = Autonomy_Final()
    while not rospy.is_shutdown():
        print("Let's start")
        # Initialize wall following and line following node
        rospy.init_node('wall_following_node', anonymous=True)
        rospy.init_node('line_following_node', anonymous=True)

        # Check for obstacles and line
        self.Wall_Following_object = WallFollower()
        self.Line_object = main()

        # Set velocity according to wall/line following
        if self.Wall_Following_object.wallfollow.scan != 0:
            self.Autonomy.velocity = self.Wall_Following_object.move
            self.Autonomy.Velocity_Publisher.publish(self.Autonomy.velocity)
        else:
            '''
            Shutdown wall_following_node
            Start stop sign detection
            ENTER CODE
            '''

            self.Autonomy.velocity = self.Line_object.line_follower_object.twist_object
            self.Autonomy.Velocity_Publisher.publish(self.Autonomy.velocity)

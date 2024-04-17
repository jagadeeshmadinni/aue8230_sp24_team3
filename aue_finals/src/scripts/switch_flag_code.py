#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import sys,tty,os,termios

def getkey():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        b = os.read(sys.stdin.fileno(),1).decode()
        return b
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN, old_settings)


def main():
    rospy.init_node('switch_flag_node', anonymous=True)
    switch_flag_pub = rospy.Publisher('/switch_flag', Bool, queue_size=10)  
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key_pressed = getkey()
        if(key_pressed=='s'):
            switch_flag_pub.publish(True)
        else:
            switch_flag_pub.publish(False)
        rate.sleep()

if __name__ == '__main__':
    main()

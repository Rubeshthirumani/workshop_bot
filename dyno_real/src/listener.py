#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def callback(data):
    rospy.loginfo(data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/left_ticks', Int16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()

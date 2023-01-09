#!/usr/bin/env python3

# Transform a given input pose from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

import math


from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def callback(msg):

    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]

    

    x = float(x/100)
    y = float(y/100)
    z = float(z/100)

    print("subscibing values x y z:", x,y,z)
    # if(theta1 < 0):
    #    theta1,theta2 = elbow_down_inverseKinematics(x,y) 
    #joint1_control_pub.publish(z)
    #joint2_control_pub.publish(theta1 )  
    #joint3_control_pub.publish(theta2 )
    my_pose = Pose()
    my_pose.position.x = 0.0
    my_pose.position.y = 0.4
    my_pose.position.z = 0.0
    my_pose.orientation.x = 0.0
    my_pose.orientation.y = 0.0
    my_pose.orientation.z = 0.0
    my_pose.orientation.w = 1.0
    print ("hai")
    transformed_pose = transform_pose(my_pose, "camera_frame", "robot_frame")

    print(transformed_pose)

if __name__ == '__main__':
    # Test Case
    rospy.init_node("transform_test")
    print ("TRANSFORM NODE INITIATED - PLEASE PUBLISH CO-ORDINATES (X,Y,Z)")

    #joint1_control_pub = rospy.Publisher('/control/joint1', Float64,queue_size=1)
    #joint2_control_pub = rospy.Publisher('/control/joint2', Float64,queue_size=1)
    #joint3_control_pub = rospy.Publisher('/control/joint3', Float64,queue_size=1)

    pose_sub = rospy.Subscriber('/position_x_y', Float32MultiArray, callback, queue_size=10)
    rospy.spin()
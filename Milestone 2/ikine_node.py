#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray 

def target_clbk(recived_msg):

    global x_goal, y_goal, z_goal, theta0, theta1, theta2, theta3

    x_goal = recived_msg.data[0] - 0.012
    y_goal = recived_msg.data[1]
    z_goal = recived_msg.data[2]

    # inverse kinematics calculations 
    r1 = np.sqrt(x_goal ** 2 + y_goal ** 2)
    r2 = np.sqrt(r1 ** 2 + (z_goal - d) ** 2)

    phi   = np.arctan(0.128 / 0.024)
    gamma = np.arccos((l3 ** 2 + 0.13 ** 2 - r2 ** 2) / (2 * l3 * l2))
    alpha = np.arcsin((l3 / r2) * np.sin(gamma))
    beta  = np.arctan((z_goal - d) / r1)

    # complying with our robot 
    theta1 = np.arctan(y_goal / x_goal)
    theta2 = np.pi / 2 - (theta0 + alpha + beta)
    theta3 = np.pi - phi - gamma

if __name__ == '__main__':
    
    # create and intializing node 
    rospy.init_node("ikine_node")
    
    # joint actuators 
    joint1_pub = rospy.Publisher("/joint1_position/command", Float64, queue_size = 10)
    joint2_pub = rospy.Publisher("/joint2_position/command", Float64, queue_size = 10)
    joint3_pub = rospy.Publisher("/joint3_position/command", Float64, queue_size = 10)
    rospy.Subscriber("/target_position", Float32MultiArray, target_clbk)

    x_goal, y_goal, z_goal, theta0, theta1, theta2, theta3 = 0, 0, 0, 0, 0, 0, 0

    # robot parameter 
    theta0 = np.arctan(0.024 / 0.128)
    d = 0.077
    l2 = 0.13 
    l3 = 0.25

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        # moving the robot 
        joint1_pub.publish( Float64(theta1) )
        joint2_pub.publish( Float64(theta2) )
        joint3_pub.publish( Float64(theta3) )

        # publishing the message 
        rospy.loginfo("-------------------------------------------------------------------------------------")
        rospy.loginfo("Calculated Joint Angles")
        print(theta1, theta2, theta3)
        rate.sleep()

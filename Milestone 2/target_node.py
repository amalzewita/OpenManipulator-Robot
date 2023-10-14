#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray 

if __name__ == '__main__':
    
    # create and intializing node 
    rospy.init_node("target_node")

    # position publisher 
    target_pub = rospy.Publisher("/target_position", Float32MultiArray, queue_size = 10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        # getting the cordination 
        target_msg = Float32MultiArray()
    
        # publishing the message 
        target_msg.data = [0.169, 0.269, 0.169]
        target_pub.publish(target_msg)
        rospy.loginfo("Target Goal [X Y Z]")
        rospy.loginfo(np.array(target_msg.data).astype(np.float16))
        rate.sleep()
        

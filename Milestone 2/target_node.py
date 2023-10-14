#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray 

if __name__ == '__main__':
    
    # Initialize the ROS node named "target_node"
    rospy.init_node("target_node")

    # Set up a ROS publisher named "/target_position" which will publish messages of type Float32MultiArray
    target_pub = rospy.Publisher("/target_position", Float32MultiArray, queue_size = 10)

    # Set the rate of publishing to 10Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        # Create a new message of type Float32MultiArray
        target_msg = Float32MultiArray()
    
        # Set the data of the message to be the desired target position [X, Y, Z]
        target_msg.data = [0.169, 0.269, 0.169]
        
        # Publish the desired target position message to the "/target_position" topic
        target_pub.publish(target_msg)
        
        # Log the target goal for user feedback
        rospy.loginfo("Target Goal [X Y Z]")
        rospy.loginfo(np.array(target_msg.data).astype(np.float16))
        
        # Sleep to maintain the loop rate of 10Hz
        rate.sleep()

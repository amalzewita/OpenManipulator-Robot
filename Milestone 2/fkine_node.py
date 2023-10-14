#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import JointState 

# Define a function that calculates the standard Denavit-Hartenberg matrix given the DH parameters
def std_DH(theta, d, a, alpha):
    DH = np.array([ [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                    [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                    [0,             np.sin(alpha),                  np.cos(alpha),                                 d],
                    [0,             0,                              0,                                             1] ])
   
    return DH

# Define a callback function to handle joint states messages
def joints_clbk(recived_msg):
    global theta1, theta2, theta3, theta4, FinalMat # Declare global variables to be used inside the function
    # Extract joint angles from the received message
    theta1 = recived_msg.position[2]
    theta2 = recived_msg.position[3]
    theta3 = recived_msg.position[4]
    theta4 = recived_msg.position[5]

if __name__ == '__main__':
    # Initialize the joint angles and transformation matrix to zero
    theta0, theta1, theta2, theta3, theta4, FinalMat = 0, 0, 0, 0, 0, 0

    # Initialize the ROS node
    rospy.init_node("fkine_node")

    # Define a translation matrix for the offset of the first joint relative to the origin 
    translation = np.array([ [1, 0, 0, 0.012],
                             [0, 1, 0,     0],
                             [0, 0, 1,     0],
                             [0, 0, 0,     1] ])

    # Create a ROS publisher for the robot's position
    position_pub = rospy.Publisher("/position_robot", Float32MultiArray, queue_size = 10)

    # Subscribe to the /joint_states topic to get the actual joint angles
    rospy.Subscriber("/joint_states", JointState, joints_clbk)

    # Calculate the angle between the 2nd and 3rd joint
    theta0 = np.arctan2(0.024, 0.128)

    # Set the loop rate for the main loop
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Calculate the transformation matrices using the standard DH method for each joint
        T01 = std_DH (theta1,                         0.077,         0,     np.pi / 2)
        T12 = std_DH (-theta2 - theta0 + (np.pi / 2),     0,     0.130,             0)
        T23 = std_DH (-theta3 + theta0 - (np.pi / 2),     0,     0.124,             0)
        T3H = std_DH (-theta4,                            0,     0.126,             0)

        # Compute the total transformation matrix from the base to the end effector 
        FinalMat = translation @ T01 @ T12 @ T23 @ T3H

        # Extract the position (x, y, z) from the transformation matrix
        pos_msg = Float32MultiArray()
        x = FinalMat[0, 3]
        y = FinalMat[1, 3]
        z = FinalMat[2, 3]

        # Calculate the orientation (roll, pitch, yaw) from the transformation matrix
        roll  = np.arctan2( FinalMat[2, 1],                                     FinalMat[2, 2])
        pitch = np.arctan2(-FinalMat[2, 0], np.sqrt(FinalMat[2, 1] ** 2 + FinalMat[2, 2] ** 2))
        yaw   = np.arctan2( FinalMat[1, 0],                                     FinalMat[0, 0])
        pose  = [x, y, z, roll, pitch, yaw]

        # Publish the robot's position and orientation
        pos_msg.data = pose
        position_pub.publish(pos_msg)
        rospy.loginfo("-------------------------------------------------------------------------------------")
        rospy.loginfo("Actual Robot Position [X Y Z Roll Pitch Yaw]")
        rospy.loginfo(np.array(pose).astype(np.float16))
        rate.sleep()  # Sleep to maintain the loop rate

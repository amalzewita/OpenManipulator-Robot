import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray

# Function to compute the Standard Denavit-Hartenberg matrix for robotic transformations.
# Inputs: 
# theta - joint angle
# d - link offset
# a - link length
# alpha - link twist
def std_DH (theta, d, a, alpha):
    DH = np.array([ [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                    [np.sin(theta), np.cos(theta) * np.cos(alpha), - np.cos(theta) * np.sin(alpha), a * np.sin(theta)], 
                    [0, np.sin(alpha), np.cos(alpha), d], [0, 0, 0,1] ])
    return DH

# Defining global variables for joint angles and final transformation matrix.
global theta1, theta2, theta3, theta4, FinalMat 

if __name__ == '__main__':
    # Initializing joint angles and transformation matrix with zeros.
    theta0, theta1, theta2, theta3, theta4, FinalMat = 0, 0, 0, 0, 0, 0 
    
    # Initializing ROS node for forward kinematics calculations.
    rospy.init_node("Forward_Kinematics")

    # Defining the translation matrix for the offset of the first joint from the origin.
    translation = np.array([[1, 0, 0, 0.012],
                            [0, 1, 0,     0],
                            [0, 0, 1,     0],
                            [0, 0, 0,     1] ])
    
    # Initializing ROS publishers for robot position and individual joint positions.
    position_pub = rospy.Publisher("/position_robot", Float32MultiArray, queue_size = 10)
    joint1_pub = rospy.Publisher("/joint1_position/command", Float64, queue_size = 10)
    joint2_pub = rospy.Publisher("/joint2_position/command", Float64, queue_size = 10)
    joint3_pub = rospy.Publisher("/joint3_position/command", Float64, queue_size = 10)
    joint4_pub = rospy.Publisher("/joint4_position/command", Float64, queue_size = 10)

    # Setting initial joint positions to 0.
    joint1_pub.publish( Float64(0) ) 
    joint2_pub.publish( Float64(0) ) 
    joint3_pub.publish( Float64(0) ) 
    joint4_pub.publish( Float64(0) )

    # Calculating the angle between the 2nd and 3rd joint.
    theta0 = np.arctan2(0.024, 0.128)

    # Setting desired joint angles for movement.
    theta1 =  np.pi / 2 
    theta2 = -np.pi / 3 
    theta3 =  np.pi / 4 
    theta4 =  np.pi / 6 

    rate = rospy.Rate(10)

    # Main loop to continuously move the robot and calculate its position.
    while not rospy.is_shutdown():
        # Publishing desired joint angles.
        joint1_pub.publish( Float64(theta1) ) 
        joint2_pub.publish( Float64(theta2) ) 
        joint3_pub.publish( Float64(theta3) ) 
        joint4_pub.publish( Float64(theta4) )

        # Computing Denavit-Hartenberg matrices for each joint.
        T01 = std_DH (theta1, 0.077, 0, np.pi / 2)
        T12 = std_DH (-theta2 - theta0 + (np.pi / 2), 0, 0.130, 0)
        T23 = std_DH (-theta3 + theta0 - (np.pi / 2), 0, 0.124, 0)
        T3H = std_DH (-theta4, 0, 0.126, 0)

        # Computing the overall transformation matrix from the end-effector to the base.
        FinalMat = translation @ T01 @ T12 @ T23 @ T3H
        
        # Extracting end-effector position from the transformation matrix.
        x = FinalMat[0, 3]
        y = FinalMat[1, 3]
        z = FinalMat[2, 3]

        # Computing end-effector orientation (roll, pitch, yaw) from the transformation matrix.
        roll = np.arctan2( FinalMat[2, 1], FinalMat[2, 2])
        pitch = np.arctan2(-FinalMat[2, 0], np.sqrt(FinalMat[2, 1] ** 2 + FinalMat[2, 2]))
        yaw = np.arctan2( FinalMat[1, 0], FinalMat[0, 0])
        pose  = [x, y, z, roll, pitch, yaw]

        # Publishing the computed end-effector position and orientation.
        pos_msg.data = pose
        position_pub.publish(pos_msg)
        
        # Logging the end-effector position and orientation for debugging.
        rospy.loginfo("Robot Position [x y z roll pitch yaw]") 
        rospy.loginfo(np.array(pose).astype(np.float16)) 
        
        rate.sleep()

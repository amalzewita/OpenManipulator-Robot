# Project Milestone 1 - Forward Kinematics with ROS and Denavit-Hartenberg (DH) Parameters

In this milestone, teams will configure joint angles to maneuver a robot, calculate the end effector's position, and publish it using ROS.

**Table of Contents**

1. [Project Milestone 1 - Forward Kinematics with ROS and Denavit-Hartenberg (DH) Parameters](#project-milestone-1---forward-kinematics-with-ros-and-denavit-hartenberg-dh-parameters)
    - [Objectives](#objectives)
    - [CODE File](#code-file)
        - [Libraries & Imports](#libraries--imports)
        - [Functions](#functions)
        - [Global Variables](#global-variables)
        - [Main Execution](#main-execution)
        - [Key Points](#key-points)
    - [Usage](#usage)
    - [Feedback and Contributions](#feedback-and-contributions)

## Objectives

1. Decide on a set of 4 reasonable joint angles ensuring **all joints are moved**.
2. Move the robot based on these joint angles using a ROS node that publishes on the joint command topics.
3. Compute the DH parameters and integrate the forward kinematics matrix up to the base of the end effector in the code.
4. Calculate the end effector position based on the selected angles.
5. Publish the end effector's position to `std_msgs/Float32MultiArray`, with data in the format: `[x y z roll pitch yaw]`.

> **Hint**: You'll need to research how to convert a 3x3 rotation matrix to Euler rotation angles (roll, pitch, yaw).

## CODE File

This script demonstrates forward kinematics for a robotic manipulator using the ROS framework and the Denavit-Hartenberg (DH) parameterization. Below is a detailed breakdown:

### Libraries & Imports:

- `rospy`: The main ROS Python library, which provides a way for Python programs to interface with ROS.
- `numpy`: Python library used for numerical calculations.
- `std_msgs.msg`: Standard ROS message types.

### Functions:

- `std_DH(theta, d, a, alpha)`: This function returns the 4x4 transformation matrix as per the standard Denavit-Hartenberg (DH) convention. The function takes in four parameters: joint angle (`theta`), link offset (`d`), link length (`a`), and link twist (`alpha`).

### Global Variables:

- `theta1, theta2, theta3, theta4`: Joint angles for the robotic manipulator.
- `FinalMat`: The transformation matrix from the end effector to the robot's base.

### Main Execution:

1. Initialization:
   - All joint angles and the final transformation matrix `FinalMat` are initialized to zero.
   - A new ROS node named `Forward_Kinematics` is initialized.
   - A translation matrix to represent the offset of the first joint from the origin is defined.
   - ROS publishers for robot's position and joint commands are created and initialized.

2. Predefined Motion:
   - The joint angles are given predefined values.
   - These angles are then published, essentially moving the robot to the predefined position.

3. Continuous Loop (`while not rospy.is_shutdown()`):
   - In this loop, the joint angles are continuously published, keeping the robot in the predefined position.
   - The DH transformation matrices for each link of the robot are calculated using the `std_DH` function.
   - The final transformation matrix from the end effector to the robot's base is obtained by multiplying the individual transformation matrices.
   - The robot's position (`x, y, z`) and orientation (`roll, pitch, yaw`) are extracted from the final transformation matrix.
   - The position and orientation values are packed into a ROS message and published.
   - The loop sleeps for a duration set by `rospy.Rate(10)` (10Hz) and then continues.

### Key Points:
- The script employs the standard DH parameterization to compute forward kinematics, thus providing the end effector's position and orientation in relation to the robot's base.
- This information can be essential in various robotic tasks such as motion planning, control, and simulation.
- The calculated position and orientation are published to the `/position_robot` topic in ROS, making it available for other ROS nodes or systems.

---

**Usage**: To utilize this script, ensure you have ROS setup and initialize the script within a ROS environment. Ensure any dependencies, such as robot descriptions or other ROS packages, are available and correctly set up.

## Feedback and Contributions

Please open an issue if you encounter any problems or have suggestions for this project. Pull requests are also welcomed!

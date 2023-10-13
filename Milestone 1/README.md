# Project Milestone 1 - Robot Kinematics in ROS

In this milestone, teams will configure joint angles to maneuver a robot, calculate the end effector's position, and publish it using ROS.

## Objectives

1. Decide on a set of 4 reasonable joint angles ensuring **all joints are moved**.
2. Move the robot based on these joint angles using a ROS node that publishes on the joint command topics.
3. Compute the DH parameters and integrate the forward kinematics matrix up to the base of the end effector in the code.
4. Calculate the end effector position based on the selected angles.
5. Publish the end effector's position to `std_msgs/Float32MultiArray`, with data in the format: `[x y z roll pitch yaw]`.

> **Hint**: You'll need to research how to convert a 3x3 rotation matrix to Euler rotation angles (roll, pitch, yaw).

## Feedback and Contributions

Please open an issue if you encounter any problems or have suggestions for this project. Pull requests are also welcomed!

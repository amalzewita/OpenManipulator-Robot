# Project Milestone 2 - Custom Kinematics with Open Manipulator

In this milestone, we delve into the core kinematic computations for the Open Manipulator in the Gazebo environment. Teams are tasked with developing forward and inverse kinematics nodes using ROS.

## Project Overview

Using the resources provided, such as packages, scripts, and lab recordings, each team should submit a working ROS package named `open_manipulator_custom_kinematics` containing two nodes. For all tasks, ensure the robot is spawned in Gazebo.

1. **Forward Kinematics Node (`fkine_node`)**:
   - Subscribes to `/joint_states` (type: `sensor_msgs/JointState`) to fetch the robot's joint angles.
   - Uses the retrieved joint angles to compute the forward kinematics with the DH parameters.
   - The resulting 4x4 transformation matrix should be used to extract 6 DoF variables [x, y, z, roll, pitch, yaw].
   - These variables should be packaged into `std_msgs/Float32MultiArray` and published to `/robot_pose`.
   - Teams should verify the node output using the GUI controller against the actual robot position in the Gazebo environment.

2. **Inverse Kinematics Node (`ikine_node`)**:
   - Subscribes to `/target_goal` (type: `std_msgs/Float32MultiArray`) for target [x, y, z] coordinates.
   - Computes the inverse kinematics using the geometric method discussed in the provided tutorials.
   - For simplicity, the fourth joint angle is assumed to be fixed at zero.
   - Joint angles resulting from the IK computations should be used to maneuver the robot in Gazebo.
   - Publish the computed joint angles to the respective topics: `/joint1_position/command`, `/joint2_position/command`, and `/joint3_position/command`.

## Feedback and Contributions

For any issues or suggestions regarding this project, please raise an issue in this repository. Contributions via pull requests are also welcome!

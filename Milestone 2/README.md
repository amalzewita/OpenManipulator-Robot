# Project Milestone 2 - Custom Kinematics with Open Manipulator

In this milestone, we delve into the core kinematic computations for the Open Manipulator in the Gazebo environment. Teams are tasked with developing forward and inverse kinematics nodes using ROS.

# Table of Contents

1. [Project Milestone 2 - Custom Kinematics with Open Manipulator](#project-milestone-2---custom-kinematics-with-open-manipulator)
   - [Project Overview](#project-overview)
   - [Forward Kinematics Node (`fkine_node`)](#forward-kinematics-node-fkine_node)
   - [Inverse Kinematics Node (`ikine_node`)](#inverse-kinematics-node-ikine_node)
2. [Explanation for the Given fkine_node Code](#explanation-for-the-given-fkine_node-code)
   - [Overview](#overview)
   - [Details](#details)
     - [Imports](#imports)
     - [DH Parameters Function](#dh-parameters-function)
     - [Joint States Callback](#joint-states-callback)
     - [Main Execution](#main-execution)
   - [Usage](#usage)
3. [Explanation for the Given ikine_node Code](#explanation-for-the-given-ikine_node-code)
   - [Overview](#overview-1)
   - [Details](#details-1)
     - [Imports](#imports-1)
     - [Target Position Callback](#target-position-callback)
     - [Main Execution](#main-execution-1)
   - [Usage](#usage-1)
4. [Explanation for the Given target_node Code](#explanation-for-the-given-target_node-code)
   - [Overview](#overview-2)
   - [Details](#details-2)
     - [Imports](#imports-2)
     - [Main Execution](#main-execution-2)
   - [Usage](#usage-2)
5. [Note](#note)
6. [Feedback and Contributions](#feedback-and-contributions)

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

# Explanation for the Given fkine_node Code

## Overview:
This Python script calculates the forward kinematics of a robot given the joint angles using the Denavit-Hartenberg (DH) parameters. The script listens to a ROS topic for the robot's joint angles and publishes the calculated position and orientation of the end effector (robot's hand or tool).

## Details:

### Imports:
```python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import JointState 
```
- `rospy`: The primary Python library for ROS.
- `numpy`: A library for numerical operations in Python.
- `Float32MultiArray`: A ROS message type for arrays of 32-bit floats.
- `JointState`: A ROS message type that represents the state of a set of joints.

### DH Parameters Function:
```python
def std_DH(theta, d, a, alpha):
```
- This function calculates the standard Denavit-Hartenberg (DH) matrix, which represents a transformation in robotics. It takes in four parameters, which are commonly used in robotic arm kinematics.

### Joint States Callback:
```python
def joints_clbk(recived_msg):
```
- A callback function that's invoked when new data is received on the `/joint_states` topic.
- This function updates the joint angles with the received values.

### Main Execution:
```python
if __name__ == '__main__':
```
- Initializes global variables representing the joint angles and the final transformation matrix.
- Initializes a ROS node named `fkine_node`.
- Defines a translation matrix to represent the offset of the first joint from the origin.
- Initializes a publisher to the `/position_robot` topic to publish the calculated end effector position.
- Subscribes to the `/joint_states` topic to get the current joint angles.
- Inside the main loop:
  - The DH matrices for each joint are computed using the `std_DH` function.
  - The final transformation matrix from the base to the end effector is computed by multiplying these matrices.
  - The position (x, y, z) of the end effector is extracted from this matrix.
  - The orientation (roll, pitch, yaw) of the end effector is calculated.
  - The position and orientation are published to the `/position_robot` topic.
  - A log message is printed showing the computed position and orientation of the end effector.

## Usage:
- To use this script in a ROS environment, you would:
  - Ensure that the robot publishes its joint states on the `/joint_states` topic.
  - Run this script, which will then calculate and publish the end effector's position and orientation on the `/position_robot` topic.
  - This script can be helpful in simulation environments or real-world scenarios where you want to know the exact position and orientation of the robot's end effector based on its joint angles.

# Explanation for the Given ikine_node Code

## Overview:
This Python script calculates the inverse kinematics of a robot given a target position for the end effector. The script listens to a ROS topic for the desired end effector position and computes the joint angles required to reach that position. It then publishes the calculated joint angles to control the robot.

## Details:

### Imports:
```python
import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray 
```
- `rospy`: The primary Python library for ROS.
- `numpy`: A library for numerical operations in Python.
- `Float64`: A ROS message type for a 64-bit float.
- `Float32MultiArray`: A ROS message type for arrays of 32-bit floats.

### Target Position Callback:
```python
def target_clbk(recived_msg):
```
- A callback function that's invoked when new data is received on the `/target_position` topic.
- This function calculates the inverse kinematics for the robot to reach the desired position. It involves trigonometry and geometry based on the robot's structure and the provided target coordinates.

### Main Execution:
```python
if __name__ == '__main__':
```
- Initializes a ROS node named `ikine_node`.
- Defines publishers for each of the three robot joints. These publishers send commands to move the robot's joints.
- Subscribes to the `/target_position` topic to get the desired position for the robot's end effector.
- Initializes several parameters related to the robot's geometry and joint angles.
- Inside the main loop:
  - The calculated joint angles are published to control the robot.
  - A log message is printed showing the calculated joint angles.

## Usage:
- To use this script in a ROS environment:
  - Ensure that your system can accept joint commands on the specified topics (`/joint1_position/command`, etc.).
  - Provide the target position for the robot's end effector by publishing to the `/target_position` topic.
  - Run this script, which will then calculate the joint angles needed to reach the target position and send commands to move the robot's joints.
  - This script is ideal for simulation environments or real-world scenarios where you want to control a robot based on a desired end effector position, and you know the robot's geometry.

# Explanation for the Given target_node Code

## Overview:

This Python script continuously broadcasts a fixed target position for the end effector of a robot in a ROS (Robot Operating System) environment. The target position is given as Cartesian coordinates in the format [X, Y, Z]. This node can be used in simulations or real-world applications where a robot needs a constant target point to aim for or move towards.

## Details:

### Imports:
```python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray 
```
- `rospy`: The primary Python library for ROS. It provides essential functionalities to interface with ROS topics, services, and more.
- `numpy`: A library for numerical operations in Python. Used here primarily for data type conversion.
- `Float32MultiArray`: A ROS message type for arrays of 32-bit floats.

### Main Execution:
```python
if __name__ == '__main__':
```
- The script starts its execution here if it's the main module being run.

1. **Node Initialization**:
```python
rospy.init_node("target_node")
```
   - Initializes a ROS node named `target_node`.

2. **Publisher Initialization**:
```python
target_pub = rospy.Publisher("/target_position", Float32MultiArray, queue_size = 10)
```
   - Creates a publisher named `target_pub` that will publish messages of type `Float32MultiArray` to the `/target_position` topic. The `queue_size` parameter is set to 10, indicating that up to 10 messages can be stored in the queue before they start getting dropped.

3. **Rate Initialization**:
```python
rate = rospy.Rate(10)
```
   - The rate at which the loop should execute is set to 10 Hz.

4. **Main Loop**:
```python
while not rospy.is_shutdown():
```
   - The code within this loop will run repeatedly until the ROS node is shut down.

   - **Set Target Coordinates**:
```python
target_msg = Float32MultiArray()
target_msg.data = [0.169, 0.269, 0.169]
```
      - Initializes an instance of `Float32MultiArray`.
      - The target coordinates [X, Y, Z] are set to [0.169, 0.269, 0.169].

   - **Publish and Log**:
```python
target_pub.publish(target_msg)
rospy.loginfo("Target Goal [X Y Z]")
rospy.loginfo(np.array(target_msg.data).astype(np.float16))
```
      - The target position is published to the `/target_position` topic.
      - The script logs the set target coordinates for easier monitoring.

   - **Sleep**:
```python
rate.sleep()
```
      - Ensures that the loop runs at the desired rate of 10 Hz.

## Usage:
- To use this script in a ROS environment:
  - Run the script. It will initialize the `target_node` and start publishing the set target position ([0.169, 0.269, 0.169]) to the `/target_position` topic at a rate of 10 Hz.
  - Other nodes in the ROS environment can subscribe to the `/target_position` topic to receive and process the published target position.
  - This script is beneficial when testing or simulating robot systems that rely on a fixed target position.

## Note:
This code assumes a specific robot arm structure and might need modifications based on the actual robot's design and geometry.

## Feedback and Contributions

For any issues or suggestions regarding this project, please raise an issue in this repository. Contributions via pull requests are also welcome!

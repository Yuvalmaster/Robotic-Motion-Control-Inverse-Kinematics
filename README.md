# Robot Arm Movement and Planning System 🦾 🦿

This project is a simulation of a robot arm movement and planning system. It allows users to visualize the movement of a robot arm in a 2D space, 
taking into account various obstacles and target coordinates.**

## Packages
The project utilizes the following packages:

- numpy: A package for scientific computing with Python.
matplotlib: A plotting library for creating visualizations in Python.
- mpl_toolkits.mplot3d: A package for creating 3D plots in matplotlib.

## Robot Arm Class
The robot_arm class defines the parameters and functions related to the robot arm. It includes the following components:

- Robot Constants: Defines the length of each segment of the robot arm, joint angle limits, and obstacle locations.
Configuration Generation: Generates a random robot configuration (joint angles) within the defined limits.
- Direct Kinematics: Calculates the position of the end-effector based on the given joint angles.
- Collision Checker: Checks if the end-effector is in collision with obstacles or violates joint angle limits.
Attraction and Repulsion Potentials: Calculates the attractive and repulsive forces acting on the end-effector.
Preparations

This section of the code initializes the robot arm object and sets the starting and target coordinates for the robot arm movement.

- Plotting Q-Free Space:
The code generates an array of allowed angles for the robot arm in space, avoiding collisions with obstacles. It then plots the synchronized space of motor angles in a 3D plot.

- Robot Movement:
This section demonstrates the movement planning of the robot arm from the starting coordinates to the target coordinates while avoiding obstacles. 
It plots the 2D space, including the upper and lower limits, obstacles, starting coordinates, and target coordinates. 
The movement planning is achieved using attraction and repulsion potentials to guide the end-effector towards the target while avoiding collisions.

## Add Info.
requirements.txt: This file lists the required dependencies and their versions for running the project. 
It is recommended to set up a conda virtual environment and install the dependencies using ```conda install --file requirements.txt```

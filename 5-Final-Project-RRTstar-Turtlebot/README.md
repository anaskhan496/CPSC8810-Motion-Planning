# CPSC 8810 - Motion Planning
## Final Project : PATH PLANNING FOR TURTLEBOT USING RRT*
###### Submitted by: Siddhant Srivastava, Mohammad Anas Imam Khan, Manikanda Balaji Venkatesan, Aakanksha Smriti 
###### Submission date: 03-10-2020

## Problem Statement
- Build a custom global planner that employs RRT* algorithm in Python.

- Implement this global planner to make the TurtleBot 3 Burger traverse through the generated path using RRT*.

## Methodolgy
- Part 1: **Map Generation**

- Part 2: **Edge collision and point collision**

- Part 3: **RRTSTAR algorithm**

- Part 4: **Gazebo planner for ROS**

## Instruction:
Execute `roslaunch motion_planning.launch`. This will launch `turtlebot3_world.world` and spawn the Turtlebot at `x = -0.6, y = 2.0`.

Open another terminal and execute `python rrt_star_ros.py`. This will execute the RRT* global planner.
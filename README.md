# SamiRony-VFF-Avoidance
## Project Title: Avoiding Obstacles with VFF

## Description:
This project implements a behavior-based algorithm for autonomous obstacle avoidance using the Virtual Force Field (VFF) approach. The TurtleBot moves forward while avoiding obstacles based on the VFF algorithm, which combines attractive and repulsive vectors to generate speed commands.

## Objectives:

Implement VFF Algorithm:
    Attractive Vector: Points forward to encourage the robot to move straight in the absence of obstacles.
    Repulsive Vector: Derived from laser scan data, this vector is inversely proportional to the distance of the closest obstacle.
    Result Vector: The sum of the attractive and repulsive vectors, used to calculate control commands.

Parameter Configuration:
    Configure parameters such as distance threshold, obstacle coefficient, speed, alignment threshold, and frequency using YAML configuration files.

Debugging:
    Publish visual markers in RViz2 to debug the attractive, repulsive, and result vectors.

Execution:
    Use a single-threaded executor to run the node, ensuring it executes at 20Hz.
## Usage:

Launch the Simulation:
Ensure that the necessary dependencies are installed.
Use the provided launch file to start the TurtleBot simulation and the avoidance node.


    ros2 launch vff_avoidance avoidance_vff.launch.py

## Debugging with RViz2
Visualize the attractive, repulsive, and result vectors by subscribing to the visualization_marker topic in RViz2:
- Blue: Attractive vector
- Red: Repulsive vector
- Green: Resultant vector
         
Authors: Sami,Rony

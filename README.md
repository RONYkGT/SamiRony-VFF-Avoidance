# SamiRony-VFF-Avoidance
## Project Title: Avoiding Obstacles with VFF

## Description:
This project implements a behavior-based algorithm for autonomous obstacle avoidance using the Virtual Force Field (VFF) approach. The TurtleBot moves forward while avoiding obstacles based on the VFF algorithm, which combines attractive and repulsive vectors to generate speed commands.

## Objectives:

- **Implement VFF Algorithm:**
    - Attractive Vector: Points towards the next way-point. *For simplicity purposes in this project, the vector will initially face where the bot is facing and maintain the initial orientation with the robot rotation*

   - Repulsive Vector: Derived from laser scan data, this vector is inversely proportional to the distance of all the obstacles in the `distance_threshold` range.

   - Result Vector: The sum of the attractive and repulsive vectors, used to calculate control commands. <u>(Capped at 1 unit length)</u>

    - Momentum calculation: Speed is calculated from the magnitude (length) of the result vector times the speed parameter (<u>**linear momentum is clamped at `speed` parameter for safety**</u>), the angular momentum is calculated using the arctan formula for the vector x and y.

- **Parameter Configuration:**
    Configure parameters such as `distance threshold,` `obstacle coefficient,` `max speed`, `alignment threshold`, and `frequency` using YAML configuration files.

- **Debugging:**
    Publish visual markers in `RViz2`to debug the attractive, repulsive, and result vectors.

- **Execution:**
    Use a single-threaded executor to run the node, ensuring it executes at 20Hz.

## Example:
![Screencast from 08-12-2024 08_21_02 PM (1)](https://github.com/user-attachments/assets/a1357602-358a-4afd-8515-a18d144e01b5)
RViz2 visualization showing speed clamping in action.

## Usage:

**Launch the Simulation:**
Ensure that the necessary dependencies are installed.
Use the provided launch file to start the TurtleBot simulation and the avoidance node.


    ros2 launch vff_avoidance avoidance_vff.launch.py


## Debugging with RViz2
Visualize the attractive, repulsive, and result vectors by subscribing to the visualization_marker topic in RViz2:
```bash
ros2 rviz2 rviz2
```
- Blue: Attractive vector
- Red: Repulsive vector
- Green: Resultant vector
         

## Authors
#### Rony Kaddoum
- Github: [@Rony](https://github.com/RONYkGT)

#### Sami Trad
- Github: [@Samitrad](https://github.com/Samitrad)
        

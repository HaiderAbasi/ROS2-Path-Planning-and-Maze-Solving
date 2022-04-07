# ROS2 Path Planning and Maze Solving
This part of the repository contains the actual maze solving algorithm for our robot in Path planning and Maze Solving Course . It addresses the 4 stages required for robot navigaition
- **Localizing** robot in maze
- **Mapping** the environment to create traversable graphs
- **Pathplanning** from maze entry to maze exit (*Shortest path to goal*)
- **Motionplanning** for the robot to navigate to maze exit.


## Nodes
- **Driving**: A node to drive robot using OC keyboard
- **Go to Goal** : A node to drive robot to a specified location in 2D space
- **Video Saver** : A node to save video feed from upper camera for Image Processing
- **maze_solver** : A node to drive robot and provide video feed from above camera
## Robot Navigation (Modules)
- **Bot Localization**: A module to perform localization of robot using Background Subtraction.
- **Bot Mapping** : A module to convert [(top down) maze view ==> traversable graph.]
- **Bot Path Planning** : A module to perform pathplanning from source to destination using provided methods.
  - DFS
  - DFS (Shortest)
  - Dijisktra
  - A-Star
- **Bot Motion Planning** : A module for aiding the vehicle in navigate to the desired destination (Maze Exit)
## Launch Files
- **maze_1_robot_camera.launch** : Launches
  - Gazebo
  - MazeBot with Robot States
  - Camera on the top of Maze
  - Simulated Maze 1 to be solved
- **maze_2_robot_camera.launch** : Launches
  - Gazebo
  - MazeBot with Robot States
  - Camera on the top of Maze
  - Simulated Maze 2 to be solved
- **gazebo.launch** : Launches
  - Robot inside of Gazebo with proper inertia Values
- **rviz.launch** : Launches
  - Robot with Dynamic Joint states
  - Rviz to test Joint's axis

## World Files
We have two world files both contain
- camera on top
- robot
- Different architecture maze

## Software Requirements
- Ubuntu 20.04
- ROS2 Foxy
- OpenCV 4.2.0

## Author
- Haider

## License
- MIT

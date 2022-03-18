# ROS2 Path Planning and Maze Solving
This part of the repository contains our ROS2 package for Path planning and Maze Solving Course . It contains
- A custom 3D model Robot *maze_bot*
- Maze design in Gazebo
- External Camera to visually observe the maze from Above
- Launch Files and Nodes to utilize source code of this repository


## Nodes
- **Driving**: A node to drive robot using OC keyboard
- **Go to Goal** : A node to drive robot to a specified location in 2D space
- **maze_solver** : A node to drive robot and provide video feed from above camera
- **Video Saver** : A node to save video feed from upper camera for Image Processing
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

## Author
- Luqman

## License
- MIT

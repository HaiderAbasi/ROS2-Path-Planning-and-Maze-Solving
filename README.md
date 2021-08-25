# ROS2-Path-Planning-and-Maze-Solving

#### To Run you need to perform certain things

- After cloning move into the folder "src"
 - cd **cd ROS2-Path-Planning-and-Maze-Solving/path_planning_ws/**
 - **colcon build**
 - **source install/setup.bash**
- Keep in mind this source step needs to be done on every new terminal you open until you have saved this path in your bash rc file


### Running the Launch Files
 - To run the Gazebo with maze and camera for recording run the following commands in order

  - **ros2 launch  path_planning world_gazebo.launch.py**
  - **ros2 run path_planning video_recording_node**
  This will save video into ~/home folder with the name -> output.avi
  if you perform twice it will replace the previous file

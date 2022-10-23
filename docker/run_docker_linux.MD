## ROS2-PathPlanning-Docker Setup (Windows-10):
> Author: Muhammad Luqman
#### Pre-Requisites:
- Get Docker desktop installed and running by following this [Guide](https://docs.docker.com/engine/install/ubuntu/)

#### Project Setup:
Clone this repo and then move into docker directory and run the bash files
- Note : make the bash files executable first
   ```
   chmod +x run_bash*
   ```
> For Cuda-enabled Graphic cards
   ```
   ./run_docker_gpu.bash
   ```
>Non Gpu Systems
   ```
   ./run_docker.bash
   ```
- To enter an **already running container** run,

   ```
   docker exec -it r2_pathplanning_container bash
   ```
2)  Execute Gazebo launch file
      ```
      ros2 launch maze_bot maze_1_robot_camera.launch.py
      ```
3)  Open a new terminal
   then enter into the running container by executing the following line
      ```
      docker exec -it r2_pathplanning_container bash
      ```
4) Execute the maze-solver node. 👍

   ```
   ros2 run maze_bot maze_solver
   ```
<br/>

---

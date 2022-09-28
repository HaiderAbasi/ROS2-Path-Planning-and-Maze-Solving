## ROS2-PathPlanning-Docker Setup (Windows-10):
> Author: Haider Abbasi
#### Pre-Requisites:
- Get Docker desktop installed and running by following this [Guide](https://docs.docker.com/desktop/install/windows-install/)
- Ensure WSL-2 is installed or upgraded from WSL-1 (if you have older version).
- Install VcXsrv Windows X Server from this [link](https://sourceforge.net/projects/vcxsrv/) 
   > **Required** for Graphic Applications to run on Windows **(e.g. Gazebo)**

#### Project Setup:
<!--
1) Pull Repo Docker image from [Link](MyRepo@DumboLuqman.com) or build the dockerfile at **here** using the following commmand
```
docker pull docker pull haiderabbasi333/ros2-pathplanning-course
```
 -->
1) Run the docker image using the following command.
>    ( For Cuda-enabled Graphic cards ) **Faster**
```
docker run -it --name r2_pathplanning_container -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 --runtime=nvidia haiderabbasi333/ros2-pathplanning-course:1 bash
```
>   ( For Non-Cuda (Amd or CPU)      ) **Slower!**
```
docker run -it --name r2_pathplanning_container -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 haiderabbasi333/ros2-pathplanning-course:1 bash
```
-   To enter an **already running container** run,

```
docker exec -it r2_pathplanning_container bash
```
2)  Execute Gazebo launch file
```
ros2 launch maze_bot maze_1_robot_camera.launch.py
```
3)  Open a new terminal (cmd_prmpt,Windows Terminal, Powershell)
then enter into the running container by executing the following line
```
docker exec -it r2_pathplanning_container bash
```

> To build the project (again). Perform,
```
cd root/ROS2-Path-Planning-and-Maze-Solving/path_planning_ws/

colcon build
```
4) Execute the maze-solver node. 👍

```
ros2 run maze_bot maze_solver
```
<br/>

---

### Docker Commands
> Useful docker commands.
##### 1)  CONTAINERS
- Stop containers
```
sudo docker kill $(sudo docker ps -a)
```
- Delete all containers
```
docker rm $(docker ps -a -q)
```
- Start a container
```
docker start (container_id)
```
##### 2) IMAGES
- Delete all images
```
docker rmi $(docker images  -q)
```
- Removing Dangling images 
```
docker rmi $(docker images --filter "dangling=true" -q --no-trunc)
```

- Connect shell to running container
```
docker exec -it (container_id) bash
```

- Remove Dangling everything (BEWARE!)
```
docker container prune
docker image prune
```
- Run container from image-ID and name it using --name.
```
docker run --name ros2_foxy -it cd2349caf569
```
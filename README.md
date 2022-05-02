# Maze Solving using Computer Vision In ROS2

[![alt text](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/blob/master/images/cover.png)](https://youtu.be/Ejl4ZLKo3Cc "Click to Watch Intro Video on Youtube")
## Repository contents
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About-this-Repository">🤝Repository's About</a></li>
    <li><a href="#Using-this-Repository">⚡ Using this Repository</a></li>
    <li><a href="#Features">⛲Features</a></li>
    <li><a href="#Pre-Course-Requirments">🧊Pre-Course Requirments</a></li>
    <li><a href="#Notes">📗 Notes</a></li>
    <li><a href="#Instructors">👤Instructors</a></li>
    <li><a href="#Course-Coupon">💰Coupon</a></li>
    <li><a href="#license">📝License</a></li>
  </ol>
</details>


## 🤝Repository's About
---
This course is focus on Maze Solving behavior of robot In a Simulation based on ROS2. Computer Vision is the key focus with integrated important robotics algorithms of Motion Planning . The type of robot we will be using is Differential Drive Robot with a caster wheel . Course is structured with below main headings .
- Custom Robot Creation
- Gazebo and Rviz Integrations
- Localization
- Navigation
- Path Planning

From our robot to last computer vision Node ,we will create every thing from scratch . Python Object Oriented programming practices will be utilized for better development.
## ⚡ Using this Repository
----
#### **Video Demonstration**
* If the repository is not working for you. Watch the free preview video on our course page 
 Where full explaination is given on setting up this repository.Section# 1 -> Lecture#2
  * **[[How to Run the Project]](https://www.udemy.com/course/ros2-path-planning-and-maze-solving-with-computer-vision/learn/lecture/32067970#overview)**
#### **Setting Up Package**
 * Clone the repository in you Home folder
```
git clone https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving.git
```
 * Navigate into your downloaded repository
 ```
cd /path/to/ROS2-Path-Planning-and-Maze-Solving/path_planning_ws
```
* Source your ROS2 installation to build workspaces
```
$ source /opt/ros/foxy/setup.bash
```
* Perform Colcon Build ( source ~/opt.ros )
```
colcon build
```
* Source your Workspace in any terminal you open to Run files from this workspace ( Basic thing of ROS2 )
```
source ~/ROS2-Path-Planning-and-Maze-Solving/path_planning_ws/install/setup.bash
```
* (Optional for Power USERs ) Add source to this workspace into bash file
 ```
  echo "source ~/ROS2-Path-Planning-and-Maze-Solving/path_planning_ws/install/setup.bash" >> ~/.bashrc
 ```
  **NOTE:** This upper command is going to add the source file path into your ~/.bashrc file ( Only perform it ONCE and you know what you are doing).This will save your time when running things from the Workspace


## ⛲ Features
---
* **Custom Robot Integeration**<br/><br/>
  - ![alt text](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/blob/master/images/robot_model.gif)<br/><br/>
* **Drive to Goal Nodes**<br/><br/>
  - ![alt text](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/blob/master/images/nodes.gif)<br/><br/>
* **Custom World Setup**<br/><br/>
  - ![alt text](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/blob/master/images/world.gif)<br/><br/>
* **Mapping**<br/><br/>
  - ![alt text](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/blob/master/images/mapping.gif)<br/><br/>
* **Path Planning**<br/><br/>
  - ![alt text](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/blob/master/images/path_planning.gif)<br/><br/>
* **Maze Solving**<br/><br/>
  - ![alt text](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/blob/master/images/maze_solving.gif)<br/><br/>


## 🧊 Pre-Course Requirments:
---
- Ubuntu 20.04 (LTS)
- ROS2 - Foxy Fitzroy
- Python 3.6
- Opencv 4.2

## 📗 Notes
---
You can access section wise notes here -> **[PDFs](./notes)**


## 💰 Coupon
----
Udemy Discounted Course Link **[[Discounted Link]](https://www.udemy.com/course/ros2-path-planning-and-maze-solving-with-computer-vision/?couponCode=LAUNCH)**

## 👤 Instructors
---
**Muhammad Luqman**

- Website: [Robotisim](https://robotisim.com)
- Github: [Luqman.git](https://github.com/noshluk2)
- LinkedIn: [Luqman.in](https://www.linkedin.com/in/muhammad-luqman-9b227a11b/)

**Haider Abbasi**

- Github: [Haider.git](https://github.com/HaiderAbasi)
- LinkedIn: [Haider.in](https://www.linkedin.com/in/haider-najeeb-68812516a/)
## 📝 License
  ----
  Distributed under the GNU-GPL License. See `LICENSE` for more information.

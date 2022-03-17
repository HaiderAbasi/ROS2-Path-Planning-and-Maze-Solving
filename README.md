# ROS2-Path-Planning-and-Maze-Solving

## This Repo is UnderConstruction !!
## 🤝 Introduction <!-- omit in toc -->
<!--  read me inspired from   : https://github.com/kartben/artificial-nose/blob/master/README.md-->
## Repository contents

* 💰 [Hardware used](./bom/README.md) ;
* ⚡ [Connections](./schematics/README.md) ;
* 👩‍💻 [Code](./firmware) ;
* 🧊 [Models](./enclosure/README.md).


## Author <!-- omit in toc -->

👤 **Benjamin Cabé**

- Website: [https://google.com](https://lawaras.com)
- Github: [@cripple](https://github.com/abbasi)
- LinkedIn: [@abbasi](https://linkedin.com/in/abbasi)

👤 **Muhammad Luqman**

- Website: [https://robotisim.com](https://robotisim.com)
- Github: [@luqman](https://github.com/larka)
- LinkedIn: [@luqman](https://linkedin.com/in/larka)

### TODO : 
#### ROS : Not Working on Other Systems

#### To Run you need to perform certain things

- After cloning move into the folder "src"
 - cd **cd ROS2-Path-Planning-and-Maze-Solving/path_planning_ws/**
 - **colcon build**
 - **source install/setup.bash**
- Keep in mind this source step needs to be done on every new terminal you open until you have saved this path in your bash rc file


### Running the Launch Files
 - To run the Gazebo with maze and camera for recording run the following commands in order

  - **ros2 launch maze_bot maze_world.launch.py**
  - **ros2 run path_planning video_recording_node**
  This will save video into ~/home folder with the name -> output.avi
  if you perform twice it will replace the previous file

  Video Recording tab also requires **ros-foxy-gazebo-plugins**
  
  
  
  ## 📝 License <!-- omit in toc -->

Copyright &copy; 2020-3000 [Larkay](https://github.com/Bharaywalay).

This project is [MIT](/LICENSE) licensed.

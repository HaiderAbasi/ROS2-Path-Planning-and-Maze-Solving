FROM osrf/ros:foxy-desktop

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

SHELL [ "/bin/bash" , "-c" ]
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN source /opt/ros/foxy/setup.bash \
    && cd ~/ \
    && git clone https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving.git

RUN  apt install -y python3-colcon-common-extensions \
    && apt-get install -y ros-foxy-joint-state-publisher-gui \
    && apt-get install -y ros-foxy-gazebo-plugins


RUN source /opt/ros/foxy/setup.bash \
 && apt-get install -y ros-foxy-joint-state-publisher \
 && apt-get install -y ros-foxy-gazebo-ros \
 && cd ~/ROS2-Path-Planning-and-Maze-Solving/path_planning_ws \
 && colcon build

RUN echo "source ~/ROS2-Path-Planning-and-Maze-Solving/path_planning_ws/install/setup.bash" >> ~/.bashrc
RUN pip install opencv-contrib-python==4.6.0.66
RUN pip install pygame
RUN echo "All Done "


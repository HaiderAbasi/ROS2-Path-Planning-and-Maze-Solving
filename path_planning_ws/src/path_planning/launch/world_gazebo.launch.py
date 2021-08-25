import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  package_dir=get_package_share_directory('path_planning')
  world_file = os.path.join(package_dir,'worlds','maze_and_camera.world')

  return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose',world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),


        # Node(
        #         package='self_driving_car_pkg',
        #         executable='light.bash',
        #         name='Lights_installer',
        #         output='screen'),


        

    ])
import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths

def generate_launch_description():

  package_share_dir=get_package_share_directory('maze_bot')
  world_file = os.path.join(package_share_dir,'worlds','test.world')
  urdf_file = os.path.join(package_share_dir, "urdf", "maze_bot.urdf")
  model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
  
  env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

  return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['gazebo', '--verbose',world_file, '-s', 'libgazebo_ros_factory.so'],
                output="screen",
                additional_env=env,
            ), 

        Node(
                package="gazebo_ros",
                node_executable="spawn_entity.py",
                arguments=["-entity","maze_bot","-b","-file", urdf_file,],
            ),
            Node(
                package="robot_state_publisher",
                node_executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ), 

    ])
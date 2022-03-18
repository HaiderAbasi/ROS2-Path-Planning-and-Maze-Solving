import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from scripts import GazeboRosPaths
def generate_launch_description():
    package_share_dir = get_package_share_directory("maze_bot")
    urdf = os.path.join(package_share_dir, "urdf", "maze_bot.urdf")
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
    env = {
            "GAZEBO_MODEL_PATH": model_path,
        }
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf]),

        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'),


    ])
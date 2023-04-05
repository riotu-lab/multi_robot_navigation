import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

  

    return LaunchDescription([

    IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("multi_robot_navigation"), '/launch', '/spawn_multi_robots.launch.py'])
        
            ),
    IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("multi_robot_navigation"), '/launch', '/multi_robot_localization.launch.py'])
 
            ),
    IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("multi_robot_navigation"), '/launch', '/navigation_multirobot.launch.py'])

            ),    

    IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("multi_robot_navigation"), '/launch', '/rviz_config.launch.py'])

            ), 
    
    ])

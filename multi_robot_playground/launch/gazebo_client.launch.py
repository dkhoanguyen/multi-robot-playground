import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from multi_robot_playground.launch_file_generator.launch_file_generator import LaunchFileGenerator

def generate_launch_description():
    # Launch Description
    ld = LaunchDescription()
    ld.add_action(LaunchFileGenerator.prepare_gazebo_client())
    return ld

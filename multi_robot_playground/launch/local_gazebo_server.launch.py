import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from multi_robot_playground.launch_file_generator.launch_file_generator import LaunchFileGenerator

def prepare_launch(context):
    # Launch argument
    world_package_arg = DeclareLaunchArgument(
        'world_package', default_value='multi_robot_playground')
    world_path_arg = DeclareLaunchArgument(
        'world_path', default_value='worlds/empty.world')

    world_package = LaunchConfiguration('world_package')
    world_path = LaunchConfiguration('world_path')

    # Local gazebo_server
    gazebo_server = LaunchFileGenerator.prepare_local_gazebo_server(
        world_package.perform(context), world_path.perform(context))
    return [
        world_package_arg,
        world_path_arg,
        gazebo_server
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prepare_launch)
    ])

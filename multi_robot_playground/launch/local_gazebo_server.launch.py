import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def prepare_launch(context):
    # Launch arguments
    world_package_arg = DeclareLaunchArgument(
        'world_package', default_value='multi_robot_playground')
    world_path_arg = DeclareLaunchArgument(
        'world_path', default_value='worlds/empty.world')

    world_package = LaunchConfiguration('world_package')
    world_path = LaunchConfiguration('world_path')

    # Prepare gazebo server
    world = os.path.join(get_package_share_directory(
        world_package.perform(context)), world_path.perform(context))
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    return [
        world_package_arg,
        world_path_arg,
        gazebo_server
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prepare_launch)
    ])

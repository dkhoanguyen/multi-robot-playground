import os
from platform import node
import yaml

from ament_index_python.packages import get_package_share_directory

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    mrp_local_bringup_pkg = get_package_share_directory(
        'mrp_local_bringup')

    # Find multi_robot_playground
    mrp_pkg = get_package_share_directory(
        'multi_robot_playground')

    config_path = 'config/config.yml'

    with open(os.path.join(mrp_pkg, config_path), 'r') as path_stream:
        config_lists = yaml.safe_load(path_stream)

    # For each robot in the list
    # Bring up the respective servers
    robots_list = config_lists['robots']['models']
    for robot in robots_list:
        robot_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mrp_local_bringup_pkg, 'launch',
                             'mrp_local_bringup.launch.py')
            ),
            launch_arguments=[
                ('robot_name', robot['name'])],
        )
        ld.add_action(robot_bringup)

    return ld

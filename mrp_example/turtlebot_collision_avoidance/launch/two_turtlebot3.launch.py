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

    # Robot 1
    robot1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mrp_local_bringup_pkg, 'launch',
                         'mrp_local_bringup.launch.py')
        ),
        launch_arguments=[
            ('robot_name', 'robot')],
    )

    ld.add_action(robot1_bringup)

    # Robot 2
    robot2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mrp_local_bringup_pkg, 'launch',
                         'mrp_local_bringup.launch.py')
        ),
        launch_arguments=[
            ('robot_name', 'robot0')],
    )

    ld.add_action(robot2_bringup)

    # Robot 2
    robot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mrp_local_bringup_pkg, 'launch',
                         'mrp_local_bringup.launch.py')
        ),
        launch_arguments=[
            ('robot_name', 'robot1')],
    )

    ld.add_action(robot3_bringup)

    # Robot 3
    robot4_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mrp_local_bringup_pkg, 'launch',
                         'mrp_local_bringup.launch.py')
        ),
        launch_arguments=[
            ('robot_name', 'robot2')],
    )

    ld.add_action(robot4_bringup)

    return ld
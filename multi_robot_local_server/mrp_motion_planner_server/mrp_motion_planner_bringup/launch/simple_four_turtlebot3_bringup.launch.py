import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    ld = LaunchDescription()
    controller_server_pkg = get_package_share_directory(
        'controller_server')

    lifecycle_manager_pkg = get_package_share_directory(
        'mrp_lifecycle_manager')

    # Controller servers
    for i in range(0, 4):
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controller_server_pkg, 'launch',
                             'controller_server.launch.py')
            ),
            launch_arguments=[
                ('robot_name', f'robot{i}')
            ]
        ))

    # lifecycle manager
    lifecycle_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lifecycle_manager_pkg, 'launch',
                         'lifecycle_manager.launch.py')
        ),
        launch_arguments=[
            ('robot_name', '')
        ]
    )

    ld.add_action(lifecycle_manager_launch)

    return ld

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    ld = LaunchDescription()

    # This should be environment variable
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value=TextSubstitution(text="robot")
    )
    ld.add_action(robot_name_arg)

    local_bringup_pkg = get_package_share_directory(
        'mrp_local_bringup')

    settings_path = 'config/settings.yml'
    with open(os.path.join(local_bringup_pkg, settings_path), 'r') as config_stream:
        settings = yaml.safe_load(config_stream)

    server_names = []
    heartbeats = []
    # Load all servers
    for server_name, server_config in settings['servers'].items():
        server_pkg = get_package_share_directory(
            f'mrp_{server_name}_server')

        # Load servers accordingly
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(server_pkg, 'launch',
                             f'{server_name}_server.launch.py')
            ),
            launch_arguments=[
                ('robot_name', LaunchConfiguration('robot_name'))
            ]
        ))
        
        # Preparations for lifecycle_manager
        server_names.append(server_name)
        heartbeats.append(server_config['heartbeat'])

    lifecycle_manager_pkg = get_package_share_directory(
        'mrp_lifecycle_manager')
    # lifecycle manager
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lifecycle_manager_pkg, 'launch',
                         'lifecycle_manager.launch.py')
        ),
        launch_arguments=[
            ('robot_name', LaunchConfiguration('robot_name')),
            ('monitored_nodes', server_names),
            ('heartbeat_interval', str(heartbeats))
        ]
    ))

    return ld

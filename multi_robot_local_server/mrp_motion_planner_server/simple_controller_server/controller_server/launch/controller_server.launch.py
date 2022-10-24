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
    load_nodes = GroupAction(
        actions=[
            Node(
                package='controller_server',
                executable='controller_server_exec',
                output='screen',
                arguments=['--ros-args'],
                parameters=[{'controller_name': ['spotturn_controller'],
                             'controller_mapping': ['spotturn_controller::SpotturnController']
                             'robot_name': 'robot1'}]
            )
        ]
    )
    ld.add_action(load_nodes)
    return ld
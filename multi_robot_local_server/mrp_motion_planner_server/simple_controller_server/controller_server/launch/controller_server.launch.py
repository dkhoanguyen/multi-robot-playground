from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def prepare_launch(context):
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='')
    robot_name = LaunchConfiguration('robot_name')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='controller_server',
                executable='controller_server_exec',
                output='screen',
                namespace=robot_name.perform(context),
                arguments=['--ros-args'],
                parameters=[{'controller_name': ['spotturn_controller'],
                             'controller_mapping': ['spotturn_controller::SpotturnController']}]
            )
        ]
    )

    return [
        robot_name_arg,
        load_nodes
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prepare_launch)
    ])


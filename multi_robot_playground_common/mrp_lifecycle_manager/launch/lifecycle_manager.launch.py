from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_launch(context):
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='')
    robot_name = LaunchConfiguration('robot_name')

    monitored_nodes_arg = DeclareLaunchArgument(
        'monitored_nodes', default_value="['']")
    monitored_nodes = LaunchConfiguration('monitored_nodes')

    heartbeat_interval_arg = DeclareLaunchArgument(
        'heartbeat_interval', default_value=[])

    heartbeat_interval = LaunchConfiguration('heartbeat_interval')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='mrp_lifecycle_manager',
                executable='lifecycle_manager',
                output='screen',
                namespace=robot_name.perform(context),
                arguments=['--ros-args'],
                parameters=[{'node_names': monitored_nodes.perform(context)},
                            {'heartbeat_interval': heartbeat_interval.perform(context)}]
            )
        ]
    )

    return [
        robot_name_arg,
        monitored_nodes_arg,
        heartbeat_interval_arg,
        load_nodes
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prepare_launch)
    ])

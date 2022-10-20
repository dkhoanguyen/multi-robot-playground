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
    monitored_nodes = [
        "/test/test_lifecycle_node"
    ]
    heartbeat_interval = [
        float(500)
    ]

    load_nodes = GroupAction(
        actions=[
            Node(
                package='multi_robot_component_testing',
                executable='lifecycle_node',
                output='screen',
                arguments=['--ros-args']
            ),
            Node(
                package='mrp_lifecycle_manager',
                executable='lifecycle_manager',
                output='screen',
                arguments=['--ros-args'],
                parameters=[{'node_names': monitored_nodes},
                            {'heartbeat_interval': heartbeat_interval}]
            )
        ]
    )
    ld.add_action(load_nodes)
    return ld

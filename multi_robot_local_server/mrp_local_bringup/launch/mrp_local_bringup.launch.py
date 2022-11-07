import os
from platform import node
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import SetParameter, Node


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch configurations
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')

    declare_robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value='robot')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_auto_start_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false')
    declare_container_name_arg  = DeclareLaunchArgument(
        'container_name',
        default_value='servers_container')
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    #
    local_bringup_pkg = get_package_share_directory(
        'mrp_local_bringup')

    settings_path = 'config/settings.yml'
    with open(os.path.join(local_bringup_pkg, settings_path), 'r') as config_stream:
        settings = yaml.safe_load(config_stream)

    # This is for lifecycle manager
    components_list = [
        'motion_planner'
    ]
    heartbeat_list = []

    # Preparing to load node
    node_actions = [
        SetParameter("use_sim_time", use_sim_time),
    ]

    # Load components
    # Motion planner
    motion_planner_server_pkg = get_package_share_directory(
        'mrp_motion_planner_server')
    # Load parameters
    with open(os.path.join(motion_planner_server_pkg, settings_path), 'r') as config_stream:
        motion_planner_settings = yaml.safe_load(config_stream)
        heartbeat_list.append(
            motion_planner_settings['general']['health']['heartbeat'])

    node_actions.append(
        Node(
            package='mrp_motion_planner_server',
            executable='motion_planner_server_exec',
            output='screen',
            namespace=robot_name,
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[motion_planner_settings]
        )
    )

    # Finally lifecycle manager
    # Preparing monitored node lists for manager
    lifecycle_manager_pkg = get_package_share_directory(
        'mrp_lifecycle_manager')
    # Load settings
    with open(os.path.join(lifecycle_manager_pkg, settings_path), 'r') as config_stream:
        lifecycle_manager_settings = yaml.safe_load(config_stream)

    node_actions.append(
        Node(
            package='mrp_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            namespace=robot_name,
            arguments=['--ros-args'],
            parameters=[{'node_names': components_list},
                        {'heartbeat_interval': heartbeat_list},
                        lifecycle_manager_settings]))

    load_nodes = GroupAction(node_actions)

    ld.add_action(declare_robot_name_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_auto_start_arg)
    ld.add_action(declare_container_name_arg)
    ld.add_action(declare_log_level_arg)

    ld.add_action(load_nodes)
    return ld

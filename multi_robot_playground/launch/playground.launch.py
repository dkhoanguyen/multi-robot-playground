import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from multi_robot_playground.launch_file_generator.launch_file_generator import LaunchFileGenerator

def generate_launch_description():
    ld = LaunchDescription()
    multi_robot_playground_pkg = get_package_share_directory(
        'multi_robot_playground')

    # We should turn this into input arguments
    config_path = 'config/config.yml'

    with open(os.path.join(multi_robot_playground_pkg, config_path), 'r') as config_stream:
        launch_args = yaml.safe_load(config_stream)

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multi_robot_playground_pkg, 'launch',
                         'local_gazebo_server.launch.py')
        ),
        launch_arguments=[
            ('world_package', launch_args['world']['package']),
            ('world_path', launch_args['world']['path'])
        ],
    )

    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multi_robot_playground_pkg,
                         'launch', 'gazebo_client.launch.py')
        ),
    )

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    
    if LaunchFileGenerator.robot_postions_valid(launch_args['robots']['models']):
        for robot_config in launch_args['robots']['models']:
            # Spawn gazebo robot
            robot_node = LaunchFileGenerator.prepare_simulated_robot(robot_config)
            ld.add_action(robot_node)

            # Robot state publisher
            state_pub_node = LaunchFileGenerator.prepare_robot_state_publisher(robot_config)
            ld.add_action(state_pub_node)

            # Local Planner

    return ld

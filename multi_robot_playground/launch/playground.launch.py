import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf_file = os.path.join(get_package_share_directory(
        'turtlebot3_description'), 'urdf', urdf_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    for robot in launch_args['robots']['models']:
        # Spawn gazebo robot
        robot_node = launch_ros.actions.Node(
            package='multi_robot_playground',
            executable='spawn_robot',
            namespace=f'namespace_{robot["name"]}',
            name=f'gazebo_spawn_{robot["name"]}',
            output='screen',
            arguments=[
                    '--name', f'{robot["name"]}',
                    '--namespace', f'{robot["namespace"]}',
                    '-x', f'{robot["x"]}',
                    '-y', f'{robot["y"]}',
                    '-yaw', f'{robot["yaw"]}',
                    '--model_package_name', f'{robot["model_package"]}',
                    '--path_to_model', f'{robot["path_to_model"]}'
            ]
        )
        ld.add_action(robot_node)

        # Robot state publisher
        state_pub_node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=f'{robot["namespace"]}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
        ld.add_action(state_pub_node)

    return ld

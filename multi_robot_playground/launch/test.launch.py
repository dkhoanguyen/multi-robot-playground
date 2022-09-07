import os

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

    launch_args = {}
    launch_args['world_package'] = 'multi_robot_playground'
    launch_args['world_path'] = 'worlds/empty.world'

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multi_robot_playground_pkg, 'launch',
                         'local_gazebo_server.launch.py')
        ),
        launch_arguments=launch_args.items(),
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
    urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)

    # Load 2 robots
    for i in range(1):
        # Spawn gazebo robot
        robot_node = launch_ros.actions.Node(
            package='multi_robot_playground',
            executable='spawn_robot',
            namespace=f'namespace_{str(i)}',
            name=f'gazebo_spawn_robot_{str(i)}',
            output='screen',
            arguments=[
                    '--name', f'robot_{str(i)}',
                    '--namespace', f'robot_{str(i)}',
                    '-x', '0.0',
                    '-y', [str(i), '.0'],
                    '-yaw', '0.0',
                    '--model_package_name', 'modified_turtlebot3_gazebo',
                    '--path_to_model', 'models/turtlebot3_burger/model.sdf'
            ]
        )
        ld.add_action(robot_node)

        # Robot state publisher
        state_pub_node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=f'robot_{str(i)}',
            output='screen',
            parameters=[{'use_sim_time': 'true'}],
            arguments=[urdf_file],
            remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
        ld.add_action(state_pub_node)

    return ld

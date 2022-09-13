import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument


class LaunchFileGenerator(object):
    @staticmethod
    def prepare_local_gazebo_server(world_package, world_path):
        world = os.path.join(
            get_package_share_directory(world_package), world_path)
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        local_gazebo_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )

        return local_gazebo_server

    @staticmethod
    def prepare_remote_gazebo_server():
        pass

    @staticmethod
    def prepare_gazebo_client():
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        gazebo_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        )

        return gazebo_client

    @staticmethod
    def robot_postions_valid(robot_configs: list):
        for idx in range(1, len(robot_configs)):
            error = robot_configs[idx-1]['radius'] + \
                robot_configs[idx]['radius']
            if LaunchFileGenerator.is_position_overlap(robot_configs[idx-1], robot_configs[idx], error):
                return False
        return True

    @staticmethod
    def is_position_overlap(base_config, compared_config, error):
        x_base = base_config['x']
        y_base = base_config['y']

        x_compared = compared_config['x']
        y_compared = compared_config['y']

        if abs(x_base - x_compared) <= error \
                and abs(y_base - y_compared) <= error:
            return True

        return False

    @staticmethod
    def prepare_simulated_robot(robot_config: dict):
        robot_node = Node(
            package='multi_robot_playground',
            executable='spawn_robot',
            namespace=f'namespace_{robot_config["name"]}',
            name=f'gazebo_spawn_{robot_config["name"]}',
            output='screen',
            arguments=[
                    '--name', f'{robot_config["name"]}',
                    '--namespace', f'{robot_config["namespace"]}',
                    '-x', f'{robot_config["x"]}',
                    '-y', f'{robot_config["y"]}',
                    '-yaw', f'{robot_config["yaw"]}',
                    '--model_package_name', f'{robot_config["model_package"]}',
                    '--path_to_model', f'{robot_config["path_to_model"]}'
            ]
        )

        return robot_node

    @staticmethod
    def prepare_robot_state_publisher(robot_config: dict):
        urdf_file = os.path.join(get_package_share_directory(
            robot_config['urdf_package']), robot_config['urdf_path'])

        use_sim_time = LaunchConfiguration('use_sim_time', default='True')

        # Robot state publisher
        state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=f'{robot_config["namespace"]}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'robot_description': Command(['xacro ', urdf_file])}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )

        return state_pub_node

    @staticmethod
    def prepare_local_planner():
        pass

    @staticmethod
    def prepare_global_planner():
        pass

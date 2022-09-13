import os
import rclpy
from rclpy.node import Node
import argparse
import tf_transformations
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xacro


class SpawnRobot(object):
    def __init__(self, name: str,
                 namespace: str,
                 x: float,
                 y: float,
                 yaw: float,
                 model_path: str):
        self._name = name
        self._namespace = namespace
        self._x = x
        self._y = y
        self._yaw = yaw
        self._model_path = model_path

        self._node = None
        self._client = None

    def set_node(self, node: Node):
        self._node = node

    def prepare_spawn_service_client(self):
        if not self._node:
            # Should log error here
            return
        self._client = self._node.create_client(SpawnEntity, "/spawn_entity")

        # Wait for client to be ready
        if not self._client.service_is_ready():
            self._client.wait_for_service(timeout_sec=1)

        # Create request object
        self._req = SpawnEntity.Request()
        self._req.name = self._name
        self._req.robot_namespace = self._namespace
        
        if 'xacro' in self._model_path:
            self._req.xml = xacro.process_file(self._model_path).toprettyxml(indent='  ')
        else:
            self._req.xml = open(self._model_path, 'r').read()
        self._req.initial_pose.position.x = float(self._x)
        self._req.initial_pose.position.y = float(self._y)
        self._req.initial_pose.position.z = 0.0

        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self._yaw)
        self._req.initial_pose.orientation.x = q[0]
        self._req.initial_pose.orientation.y = q[1]
        self._req.initial_pose.orientation.z = q[2]
        self._req.initial_pose.orientation.w = q[3]

    def send_request(self):
        if not self._client:
            # Should log error here
            return
        self._node.get_logger().debug("Sending service request to `/spawn_entity`")
        future = self._client.call_async(self._req)
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())


def main(args=None):
    parser = argparse.ArgumentParser(description='Spawn a robot into Gazebo')
    parser.add_argument('-n', '--name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-yaw', type=float, default=0,
                        help='the yaw component of the initial position [radians]')
    parser.add_argument('--model_package_name', type=str, default='turtlebot3_gazebo',
                        help='the name of model package of the robot')
    parser.add_argument('--path_to_model', type=str, default='models/turtlebot3_burger/model.sdf',
                        help='the relative path to the model file in the package')

    args, _ = parser.parse_known_args()
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    model_file_path = os.path.join(
        get_package_share_directory(args.model_package_name),
        args.path_to_model
    )

    robot_spawner = SpawnRobot(name=args.name,
                               namespace=args.namespace,
                               x=args.x,
                               y=args.y,
                               yaw=args.yaw,
                               model_path=model_file_path)
    robot_spawner.set_node(node)
    robot_spawner.prepare_spawn_service_client()
    robot_spawner.send_request()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState, GetState


class LifecycleManagerServiceClient(Node):
    def __init__(self):
        super().__init__('lifecycle_manager_service_client')

    def activate_lifecycle_manager(self, robot_name):
        client = self.create_client(
            ChangeState, f'/{robot_name}/lifecycle_manager/change_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = 3
        future = client.call_async(change_state_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot_name", required=True, type=str, help='Robot name')
    args = parser.parse_args()

    change_state_cli = LifecycleManagerServiceClient()
    res = change_state_cli.activate_lifecycle_manager(str(args.robot_name))
    print(res)
    change_state_cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

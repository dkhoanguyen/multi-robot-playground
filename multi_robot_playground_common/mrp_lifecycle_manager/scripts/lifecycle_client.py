#!/usr/bin/env python3
import argparse
from enum import Enum

import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState, GetState

from mrp_common_msgs.srv import MonitoredNodeArray, MonitoredNode


class LifecycleTransition(Enum):
    CONFIGURE = 'configure'
    ACTIVATE = 'activate'
    DEACTIVATE = 'deactivate'
    CLEANUP = 'cleanup'
    SHUTDOWN = 'shutdown'

    def __str__(self):
        return f'{self.value}'


class LifecycleClient(Node):
    def __init__(self, robot_name='robot'):
        super().__init__(f'{robot_name}_lifecycle_client')
        self._robot_name = robot_name

    def cleanup(self):
        self.destroy_node()

    def get_node_state(self, node_name):
        # Initialise GetState client
        get_state_client = self.create_client(
            GetState, f'{self._robot_name}/{node_name}/get_state')
        while not get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_state service not available, waiting again...')
        get_state_req = GetState.Request()

        get_state_future = get_state_client.call_async(get_state_req)
        rclpy.spin_until_future_complete(self, get_state_future)
        return get_state_future.result()

    def change_node_state(self, node_name, state):
        # ChangeState client
        change_state_client = self.create_client(
            ChangeState, f'{self._robot_name}/{node_name}/change_state')
        while not change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('change_state service not available, waiting again...')
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = state

        change_state_future = change_state_client.call_async(change_state_req)
        rclpy.spin_until_future_complete(self, change_state_future)
        return change_state_future.result()


class ManagedNodeClient(Node):
    def __init__(self, robot_name='robot'):
        super().__init__(f'{robot_name}_managed_node')
        self.robot_name = robot_name
    
    def cleanup(self):
        self.destroy_node()

    def change_node_state(self, node_name, state):
        change_state_client = self.create_client(
            MonitoredNodeArray, f'{self.robot_name}/change_monitored_nodes_state')
        while not change_state_client.wait_for_service(timeout_sec=1.0):
            self.set_logger().info("change_monitored_nodes_state not available, waiting...")
        change_state_req = MonitoredNodeArray.Request()
        # change_state_req.

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot_name", required=True, type=str, help='Robot name')
    parser.add_argument(
        "--controller", required=True, type=str, help='lifecycle, or managed_nodes')
    parser.add_argument(
        "--action", required=True, type=str, help='change_state, or get_state')
    parser.add_argument(
        "--node_name", required=True, type=str, help='Node name.')
    parser.add_argument(
        "--state", required=False, type=str, help='Transition state if change_state \
        action is used.')
    args = parser.parse_args()

    if args.controller == 'lifecycle':
        cli = LifecycleClient(args.robot_name)
    if args.action == 'get_state':
        state = cli.get_node_state(args.node_name)
    
    cli.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

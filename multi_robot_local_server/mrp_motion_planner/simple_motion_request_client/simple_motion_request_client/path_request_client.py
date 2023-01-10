#!/usr/bin/env python3

import os
import yaml
import csv
import argparse
from math import atan2

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from tf_transformations import quaternion_from_euler

class PathRequestClient(Node):
    def __init__(self, robot_name='robot'):
        super().__init__(f'{robot_name}_path_request_client')
        self._action_client = ActionClient(
            self, FollowPath, f'/{robot_name}/follow_path')

    def send_goal(self, raw_path):
        goal_msg = FollowPath.Goal()
        path = Path()

        for waypoint in raw_path:
            pose = PoseStamped()
            pose.pose.position.x = waypoint['position']['x']
            pose.pose.position.y = waypoint['position']['y']
            pose.pose.position.z = waypoint['position']['z']

            r = waypoint['orientation']['r']
            p = waypoint['orientation']['p']
            y = waypoint['orientation']['y']

            quat = quaternion_from_euler(r, p, y)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            path.poses.append(pose)

        goal_msg.path = path
        print("waiting for server")
        self._action_client.wait_for_server(1)
        print("Action server is ready")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        print("Result is here")
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.distance_to_goal))


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot_name", required=True, type=str, help='Robot name')
    args = parser.parse_args()
    
    pkg_path = get_package_share_directory('simple_motion_request_client')
    with open(os.path.join(pkg_path, 'config/path.yml'), 'r') as path_stream:
        path_list = yaml.safe_load(path_stream)
        print(path_list)
    
    paths = []
    with open(os.path.join(pkg_path, 'sample_paths/sinwave_075.csv'), 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                path = {}
                position = {}
                position['x'] = float(row[0])
                position['y'] = float(row[1])
                position['z'] = float(row[2]) 
                path['position'] = position
                orientation = {}
                orientation['r'] = 0.0
                orientation['p'] = 0.0
                orientation['y'] = 0.0
                path['orientation'] = orientation

                print(path)
                paths.append(path)
                line_count += 1
                
    action_client = PathRequestClient(args.robot_name)
    # action_client.send_goal(paths)
    action_client.send_goal(path_list['paths'][str(args.robot_name)])

    print("Done sending goal")
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

from math import atan2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose

from tf_transformations import quaternion_from_euler

class PathRequestClient(Node):
    def __init__(self, robot_name='robot'):
        super.__init__('PathRequestClient')
        self._action_client = ActionClient(
            self, FollowPath, f'/{robot_name}/motion_planner/follow_path')

    def send_goal(self, raw_path):
        goal_msg = FollowPath.Goal()
        path = Path()

        for waypoint in raw_path:
            pose = Pose()
            pose.position.x = waypoint['position']['x']
            pose.position.y = waypoint['position']['y']
            pose.position.z = waypoint['position']['z']

            r = waypoint['orientation']['r']
            p = waypoint['orientation']['p']
            y = waypoint['orientation']['y']

            quat = quaternion_from_euler(r, p, y)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            path.poses.append(pose)

        goal_msg.path = path
        self._action_client.wait_for_server(1)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.distance_to_goal))


def main(args=None):
    rclpy.init(args=args)

    # Robot 0
    robot0_waypoints = []
    robot0_waypoint1 = {}
    robot0_waypoint1['position'] = {
        'x' : 1.0,
        'y' : 1.0,
        'z' : 0.0
    }
    robot0_waypoint1['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : atan2(1.0 - 0, 1.0 - 0)
    }
    robot0_waypoints.append(robot0_waypoint1)

    robot0_waypoint2 = {}
    robot0_waypoint2['position'] = {
        'x' : 0.0,
        'y' : 0.0,
        'z' : 0.0
    }
    robot0_waypoint2['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : 0.0
    }
    robot0_waypoints.append(robot0_waypoint2)
    

    action_client = PathRequestClient()
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

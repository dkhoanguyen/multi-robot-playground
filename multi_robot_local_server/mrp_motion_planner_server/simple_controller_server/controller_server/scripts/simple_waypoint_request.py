#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from mrp_motion_planner_msgs.srv import Waypoints
from geometry_msgs.msg import Pose, PoseArray

from tf_transformations import quaternion_from_euler


class MinimalClientAsync(Node):

    def __init__(self, robot_name='robot1'):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Waypoints, f'/{robot_name}/waypoints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Waypoints.Request()

    def send_request(self, waypoints):
        pose_array = PoseArray()

        for waypoint in waypoints:
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

            pose_array.poses.append(pose)

        self.req.pose_array = pose_array
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    
    waypoint = {}
    waypoint['position'] = {
        'x' : 1.0,
        'y' : 2.0,
        'z' : 0.0
    }
    waypoint['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : 0.0
    }
    waypoints = [waypoint]
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(waypoints)
    print(response)
    # minimal_client.get_logger().info(response.success)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
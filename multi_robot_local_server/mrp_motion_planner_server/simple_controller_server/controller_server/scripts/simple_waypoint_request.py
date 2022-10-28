#!/usr/bin/env python3

from math import atan2
import time
import sys
import rclpy
from rclpy.node import Node

from mrp_motion_planner_msgs.srv import Waypoints
from geometry_msgs.msg import Pose, PoseArray

from tf_transformations import quaternion_from_euler


class WaypointRequestClient(Node):

    def __init__(self, robot_name='robot0'):
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
    
    # Robot 0
    robot0_waypoints = []
    robot0_waypoint1 = {}
    robot0_waypoint1['position'] = {
        'x' : 0.25,
        'y' : 0.75,
        'z' : 0.0
    }
    robot0_waypoint1['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : atan2(0.75 - 0, 0.25 - 0)
    }
    robot0_waypoints.append(robot0_waypoint1)

    robot0_waypoint2 = {}
    robot0_waypoint2['position'] = {
        'x' : 1.0,
        'y' : 1.0,
        'z' : 0.0
    }
    robot0_waypoint2['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : 0.0
    }
    robot0_waypoints.append(robot0_waypoint2)

    # Robot 1
    robot1_waypoints = []
    robot1_waypoint1 = {}
    robot1_waypoint1['position'] = {
        'x' : 0.6,
        'y' : 0.6,
        'z' : 0.0
    }
    robot1_waypoint1['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : atan2(0.6 - 0, 0.6 - 1)
    }
    robot1_waypoints.append(robot1_waypoint1)

    robot1_waypoint2 = {}
    robot1_waypoint2['position'] = {
        'x' : 1.0,
        'y' : 0.0,
        'z' : 0.0
    }
    robot1_waypoint2['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : 0.0
    }
    robot1_waypoints.append(robot1_waypoint2)

    # Robot 2
    robot2_waypoints = []
    robot2_waypoint1 = {}
    robot2_waypoint1['position'] = {
        'x' : 0.75,
        'y' : 0.25,
        'z' : 0.0
    }
    robot2_waypoint1['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : atan2(0.25 - 1, 0.75 - 1)
    }
    robot2_waypoints.append(robot2_waypoint1)

    robot2_waypoint2 = {}
    robot2_waypoint2['position'] = {
        'x' : 0.0,
        'y' : 0.0,
        'z' : 0.0
    }
    robot2_waypoint2['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : 0.0
    }
    robot2_waypoints.append(robot2_waypoint2)

    # Robot 3
    robot3_waypoints = []
    robot3_waypoint1 = {}
    robot3_waypoint1['position'] = {
        'x' : 0.4,
        'y' : 0.4,
        'z' : 0.0
    }
    robot3_waypoint1['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : atan2(0.4 - 1, 0.4 - 0)
    }
    robot3_waypoints.append(robot3_waypoint1)

    robot3_waypoint2 = {}
    robot3_waypoint2['position'] = {
        'x' : 0.0,
        'y' : 1.0,
        'z' : 0.0
    }
    robot3_waypoint2['orientation'] = {
        'r' : 0.0,
        'p' : 0.0,
        'y' : 0.0
    }
    robot3_waypoints.append(robot3_waypoint2)

    # Robot 1
    minimal_client1 = WaypointRequestClient(robot_name="robot1")
    response = minimal_client1.send_request(robot1_waypoints)
    minimal_client1.destroy_node()

     # Robot 3
    minimal_client3 = WaypointRequestClient(robot_name="robot3")
    response = minimal_client3.send_request(robot3_waypoints)
    minimal_client3.destroy_node()
    
    time.sleep(4)
    # Robot 0
    minimal_client0 = WaypointRequestClient(robot_name="robot0")
    response = minimal_client0.send_request(robot0_waypoints)
    minimal_client0.destroy_node()

    # Robot 0
    minimal_client2 = WaypointRequestClient(robot_name="robot2")
    response = minimal_client2.send_request(robot2_waypoints)
    minimal_client2.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
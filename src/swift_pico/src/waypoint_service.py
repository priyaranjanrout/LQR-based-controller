#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from waypoint_navigation.srv import GetWaypoints

class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.srv = self.create_service(GetWaypoints, 'waypoints', self.waypoint_callback)
        self.waypoints = [
            [ 1.40,  0.00, 2.00],
            [ 1.08,  0.44, 2.15],
            [ 0.74,  0.88, 2.30],
            [ 0.43,  1.33, 2.45],
            [-0.35,  1.18, 2.65],
            [-0.74,  1.00, 2.80],
            [-1.13,  0.83, 2.95],
            [-1.26,  0.32, 3.15],
            [-1.32,  0.06, 3.25],
            [-1.38, -0.20, 3.35],
            [-1.02, -0.73, 3.55],
            [-0.67, -1.05, 3.70],
            [-0.32, -1.37, 3.85],
            [ 0.45, -1.01, 4.05],
            [ 0.83, -0.86, 4.20],
            [ 1.21, -0.70, 4.35],
            [ 1.25, -0.15, 4.55],
            [ 1.28,  0.13, 4.65],
            [ 1.30,  0.40, 4.75],
            [ 0.74,  0.90, 4.95],
            [ 0.47,  1.14, 5.10],
            [ 0.20,  1.38, 5.25],
            [-0.53,  1.18, 5.45],
            [-0.89,  0.88, 5.60],
            [-1.25,  0.57, 5.75],
            [-1.32,  0.29, 5.95],
            [-1.36,  0.14, 6.05],
            [-1.40,  0.00, 6.15]
        ]
    
    def waypoint_callback(self, request, response):

        if request.get_waypoints == True :
            response.waypoints.poses = [Pose() for _ in range(len(self.waypoints))]
            for i in range(len(self.waypoints)):
                response.waypoints.poses[i].position.x = self.waypoints[i][0]
                response.waypoints.poses[i].position.y = self.waypoints[i][1]
                response.waypoints.poses[i].position.z = self.waypoints[i][2]
            self.get_logger().info("Incoming request for Waypoints")
            return response

        else:
            self.get_logger().info("Request rejected")

def main():
    rclpy.init()
    waypoints = WayPoints()
    try:
        rclpy.spin(waypoints)
    except KeyboardInterrupt:
        waypoints.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoints.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
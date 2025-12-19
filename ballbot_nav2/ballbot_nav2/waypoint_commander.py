#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

class WaypointCommander(Node):
    def __init__(self):
        super().__init__('waypoint_commander')
        
        self.navigator = BasicNavigator()

        self.get_logger().info('Waiting for Navigation2 to become active...')
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('Nav2 is active. Starting waypoint navigation.')

        # =============================================================
        # DEFINE YOUR WAYPOINTS HERE
        # =============================================================
        # Each waypoint needs:
        #   - x: X position in meters (map frame)
        #   - y: Y position in meters (map frame)
        #   - yaw: Rotation in radians (0 = facing +X, 1.57 = facing +Y)
        #
        # Assessment world coordinates:
        #   - Center: (0, 0)
        #   - Walls at: x = ±4.0, y = ±4.0
        #   - Pen areas: around (±0.6, 3.3)
        # =============================================================
        
        waypoints = [
            {'x': 1.0,  'y': 0.0,  'yaw': 0.0},      # Waypoint 1: 1m forward
            {'x': 2.0,  'y': 1.0,  'yaw': 1.57},     # Waypoint 2: turn left
            {'x': 0.0,  'y': 2.0,  'yaw': 3.14},     # Waypoint 3: face backward
            {'x': -1.0, 'y': 0.0,  'yaw': -1.57},    # Waypoint 4: turn right
            {'x': 0.0,  'y': 0.0,  'yaw': 0.0},      # Waypoint 5: return to start
        ]
        
        # Convert waypoints to PoseStamped messages
        goal_poses = []
        for i, wp in enumerate(waypoints):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            # Set position
            goal_pose.pose.position.x = wp['x']
            goal_pose.pose.position.y = wp['y']
            goal_pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion (simplified for 2D)
            import math
            goal_pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
            goal_pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
            
            goal_poses.append(goal_pose)
            self.get_logger().info(f'Waypoint {i+1}: x={wp["x"]}, y={wp["y"]}, yaw={wp["yaw"]:.2f} rad')

        # =============================================================
        # NAVIGATE THROUGH WAYPOINTS
        # =============================================================
        
        self.get_logger().info(f'Starting navigation through {len(goal_poses)} waypoints...')
        self.navigator.followWaypoints(goal_poses)

        # Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                current_wp = feedback.current_waypoint
                self.get_logger().info(f'Executing waypoint {current_wp + 1}/{len(goal_poses)}')

        # Check result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('All waypoints completed successfully!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Waypoint navigation was canceled.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Waypoint navigation failed.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCommander()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
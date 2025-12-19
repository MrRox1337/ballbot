#!/usr/bin/env python3

"""
Simple Commander for Ballbot Navigation.
This node uses the Nav2 Simple Commander API to trigger movement
after the AMCL lifecycle is activated via auto-initialization.
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

class SimpleCommander(Node):
    def __init__(self):
        # Initialize the Node class
        super().__init__('simple_commander')
        
        # Initialize the BasicNavigator
        self.navigator = BasicNavigator()

        # Wait for Navigation2 to launch and be ready
        # This relies on AMCL having an initial pose set in the params file
        self.get_logger().info('Waiting for Navigation2 to become active...')
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('Nav2 is active. Preparing to send goal.')

        # Define the goal pose 1 meter forward
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.w = 1.5

        # Send the robot to the goal
        self.get_logger().info('Sending goal request...')
        self.navigator.goToPose(goal_pose)

        # Loop until the task is finished
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # Log progress occasionally
                self.get_logger().info(
                    f'Estimated distance to goal: {feedback.distance_remaining:.2f} m'
                )

        # Evaluate the result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation Succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation was Canceled.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation Failed.')
        else:
            self.get_logger().info('Goal has an invalid return status.')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCommander()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Relay node to convert Twist to TwistStamped for diff_drive controller.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to velocity smoother's output
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_smoothed',  # LISTEN TO THIS
            self.cmd_vel_callback,
            10
        )
        
        # Publish to diff_drive controller
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            10
        )
        
        self.get_logger().info('Cmd Vel Relay: /cmd_vel_smoothed -> /diff_drive_base_controller/cmd_vel')

    def cmd_vel_callback(self, msg):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_footprint'
        stamped_msg.twist = msg
        self.publisher.publish(stamped_msg)
        
        # Debug output
        # self.get_logger().info(f'Relaying: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
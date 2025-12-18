#!/usr/bin/env python3
"""
IMU Complementary Filter for Ballbot
Provides additional filtering and fusion similar to BNO055's internal algorithms

This node applies a complementary filter to combine:
- Gyroscope data (for short-term accuracy)
- Accelerometer data (for long-term stability)

Optional: Can be used for additional smoothing beyond the imu_processor node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import numpy as np


class IMUFilter(Node):
    """
    Complementary filter for IMU data fusion.
    Emulates the sensor fusion done internally by BNO055.
    """
    
    def __init__(self):
        super().__init__('imu_filter')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_filtered')
        self.declare_parameter('filter_alpha', 0.98)  # Complementary filter coefficient
        self.declare_parameter('update_rate', 100.0)  # Hz
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.alpha = self.get_parameter('filter_alpha').value
        self.dt = 1.0 / self.get_parameter('update_rate').value
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            10
        )
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, output_topic, 10)
        
        # Filter state
        self.last_time = None
        self.filtered_orientation = None
        self.gyro_angle = np.array([0.0, 0.0, 0.0])  # Roll, Pitch, Yaw from gyro integration
        
        self.get_logger().info(f'IMU Complementary Filter started')
        self.get_logger().info(f'  Alpha: {self.alpha} (higher = trust gyro more)')
        
    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]
        q[3] = cr * cp * cy + sr * sp * sy  # w
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        
        return q
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def imu_callback(self, msg):
        """
        Apply complementary filter to IMU data.
        
        Complementary filter:
        - Use gyroscope for short-term accuracy (no drift in short term)
        - Use accelerometer for long-term correction (drifts less over time)
        
        filtered_angle = alpha * (prev_angle + gyro * dt) + (1 - alpha) * accel_angle
        """
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            # First message - initialize
            self.filtered_orientation = msg.orientation
            self.gyro_angle = self.quaternion_to_euler(msg.orientation)
            self.last_time = current_time
            
            # Publish initial message as-is
            self.imu_pub.publish(msg)
            return
        
        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Get current measurements
        accel_euler = self.quaternion_to_euler(msg.orientation)
        gyro_rates = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Integrate gyroscope (high-pass: good short-term, drifts long-term)
        self.gyro_angle += gyro_rates * dt
        
        # Complementary filter for roll and pitch
        # (Yaw typically comes from magnetometer in BNO055, but we'll use gyro)
        filtered_euler = np.zeros(3)
        
        for i in range(3):
            # Combine gyro integration with accelerometer/orientation
            filtered_euler[i] = (
                self.alpha * self.gyro_angle[i] + 
                (1.0 - self.alpha) * accel_euler[i]
            )
            
            # Normalize angles
            filtered_euler[i] = self.normalize_angle(filtered_euler[i])
        
        # Update gyro angle for next iteration
        self.gyro_angle = filtered_euler.copy()
        
        # Convert back to quaternion
        q = self.euler_to_quaternion(filtered_euler[0], filtered_euler[1], filtered_euler[2])
        
        # Create filtered IMU message
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.stamp = current_time.to_msg()
        
        # Set filtered orientation
        filtered_msg.orientation.x = q[0]
        filtered_msg.orientation.y = q[1]
        filtered_msg.orientation.z = q[2]
        filtered_msg.orientation.w = q[3]
        filtered_msg.orientation_covariance = msg.orientation_covariance
        
        # Pass through angular velocity (already good from sensor)
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        # Pass through linear acceleration
        filtered_msg.linear_acceleration = msg.linear_acceleration
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # Publish filtered data
        self.imu_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
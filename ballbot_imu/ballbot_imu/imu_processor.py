#!/usr/bin/env python3
"""
IMU Processor Node for Ballbot
Emulates a BNO055 Absolute Orientation Sensor

This node subscribes to the raw Gazebo IMU data and processes it to provide:
- Absolute orientation (quaternion)
- Euler angles (roll, pitch, yaw)
- Angular velocities
- Linear accelerations (with gravity compensation option)

Similar to how a real BNO055 sensor would operate.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, Vector3
import math


class IMUProcessor(Node):
    """
    Processes raw IMU data from Gazebo and publishes filtered/processed data.
    Emulates BNO055 sensor behavior with absolute orientation.
    """
    
    def __init__(self):
        super().__init__('imu_processor')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/imu_raw')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('euler_topic', '/imu/euler')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('remove_gravity', False)  # BNO055 can compensate gravity
        self.declare_parameter('noise_stddev_orientation', 0.001)  # Simulated sensor noise
        self.declare_parameter('noise_stddev_angular_vel', 0.002)
        self.declare_parameter('noise_stddev_linear_accel', 0.01)
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        euler_topic = self.get_parameter('euler_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.remove_gravity = self.get_parameter('remove_gravity').value
        
        # Noise parameters (simulate real sensor characteristics)
        self.noise_orient = self.get_parameter('noise_stddev_orientation').value
        self.noise_gyro = self.get_parameter('noise_stddev_angular_vel').value
        self.noise_accel = self.get_parameter('noise_stddev_linear_accel').value
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            10
        )
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, output_topic, 10)
        self.euler_pub = self.create_publisher(Vector3Stamped, euler_topic, 10)
        self.quat_pub = self.create_publisher(QuaternionStamped, '/imu/quaternion', 10)
        
        # State variables
        self.last_orientation = None
        self.gravity = 9.81  # m/s^2
        
        self.get_logger().info(f'IMU Processor started')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Euler: {euler_topic}')
        self.get_logger().info(f'  Gravity removal: {self.remove_gravity}')
        
    def quaternion_to_euler(self, q):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
        Uses the same convention as BNO055 sensor.
        
        Args:
            q: Quaternion (x, y, z, w)
            
        Returns:
            (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def remove_gravity_component(self, linear_accel, orientation):
        """
        Remove gravity component from linear acceleration.
        BNO055 can provide both raw and gravity-compensated acceleration.
        
        Args:
            linear_accel: Vector3 of linear acceleration
            orientation: Quaternion representing orientation
            
        Returns:
            Vector3 of gravity-compensated linear acceleration
        """
        # Convert quaternion to rotation matrix to get gravity direction
        q = orientation
        
        # Gravity vector in world frame (pointing down)
        # After rotation, we get gravity in body frame
        gx = 2.0 * (q.x * q.z - q.w * q.y) * self.gravity
        gy = 2.0 * (q.w * q.x + q.y * q.z) * self.gravity
        gz = (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * self.gravity
        
        # Remove gravity from acceleration
        compensated = Vector3()
        compensated.x = linear_accel.x - gx
        compensated.y = linear_accel.y - gy
        compensated.z = linear_accel.z - gz
        
        return compensated
    
    def add_sensor_noise(self, value, stddev):
        """
        Add Gaussian noise to simulate real sensor characteristics.
        BNO055 has typical noise levels that we emulate here.
        """
        import random
        return value + random.gauss(0, stddev)
    
    def imu_callback(self, msg):
        """
        Process incoming IMU data from Gazebo simulation.
        Emulates BNO055 sensor processing and output.
        """
        # Create processed IMU message
        processed_imu = Imu()
        processed_imu.header.stamp = self.get_clock().now().to_msg()
        processed_imu.header.frame_id = self.frame_id
        
        # === ORIENTATION (Absolute) ===
        # BNO055 provides absolute orientation via sensor fusion
        # In sim, we get this directly from Gazebo
        processed_imu.orientation = msg.orientation
        
        # Add realistic orientation noise
        processed_imu.orientation.x = self.add_sensor_noise(msg.orientation.x, self.noise_orient)
        processed_imu.orientation.y = self.add_sensor_noise(msg.orientation.y, self.noise_orient)
        processed_imu.orientation.z = self.add_sensor_noise(msg.orientation.z, self.noise_orient)
        processed_imu.orientation.w = self.add_sensor_noise(msg.orientation.w, self.noise_orient)
        
        # Normalize quaternion after adding noise
        norm = math.sqrt(
            processed_imu.orientation.x**2 + 
            processed_imu.orientation.y**2 + 
            processed_imu.orientation.z**2 + 
            processed_imu.orientation.w**2
        )
        if norm > 0:
            processed_imu.orientation.x /= norm
            processed_imu.orientation.y /= norm
            processed_imu.orientation.z /= norm
            processed_imu.orientation.w /= norm
        
        # Set orientation covariance (based on BNO055 specs)
        # BNO055 typical orientation accuracy: ±1° = ±0.0175 rad
        orient_variance = (0.0175) ** 2
        processed_imu.orientation_covariance = [
            orient_variance, 0.0, 0.0,
            0.0, orient_variance, 0.0,
            0.0, 0.0, orient_variance
        ]
        
        # === ANGULAR VELOCITY ===
        # Add gyroscope noise
        processed_imu.angular_velocity.x = self.add_sensor_noise(msg.angular_velocity.x, self.noise_gyro)
        processed_imu.angular_velocity.y = self.add_sensor_noise(msg.angular_velocity.y, self.noise_gyro)
        processed_imu.angular_velocity.z = self.add_sensor_noise(msg.angular_velocity.z, self.noise_gyro)
        
        # Set angular velocity covariance (based on BNO055 specs)
        # BNO055 gyro noise: 0.014 °/s = 0.000244 rad/s
        gyro_variance = (0.000244) ** 2
        processed_imu.angular_velocity_covariance = [
            gyro_variance, 0.0, 0.0,
            0.0, gyro_variance, 0.0,
            0.0, 0.0, gyro_variance
        ]
        
        # === LINEAR ACCELERATION ===
        if self.remove_gravity:
            # BNO055 mode: Linear Acceleration (gravity removed)
            processed_imu.linear_acceleration = self.remove_gravity_component(
                msg.linear_acceleration, 
                msg.orientation
            )
        else:
            # BNO055 mode: Raw Acceleration (with gravity)
            processed_imu.linear_acceleration = msg.linear_acceleration
        
        # Add accelerometer noise
        processed_imu.linear_acceleration.x = self.add_sensor_noise(
            processed_imu.linear_acceleration.x, self.noise_accel
        )
        processed_imu.linear_acceleration.y = self.add_sensor_noise(
            processed_imu.linear_acceleration.y, self.noise_accel
        )
        processed_imu.linear_acceleration.z = self.add_sensor_noise(
            processed_imu.linear_acceleration.z, self.noise_accel
        )
        
        # Set linear acceleration covariance (based on BNO055 specs)
        # BNO055 accelerometer noise: 150 µg RMS = 0.00147 m/s²
        accel_variance = (0.00147) ** 2
        processed_imu.linear_acceleration_covariance = [
            accel_variance, 0.0, 0.0,
            0.0, accel_variance, 0.0,
            0.0, 0.0, accel_variance
        ]
        
        # Publish processed IMU data
        self.imu_pub.publish(processed_imu)
        
        # === PUBLISH EULER ANGLES ===
        # BNO055 also provides Euler angle output
        roll, pitch, yaw = self.quaternion_to_euler(processed_imu.orientation)
        
        euler_msg = Vector3Stamped()
        euler_msg.header = processed_imu.header
        euler_msg.vector.x = math.degrees(roll)   # Convert to degrees like BNO055
        euler_msg.vector.y = math.degrees(pitch)
        euler_msg.vector.z = math.degrees(yaw)
        
        self.euler_pub.publish(euler_msg)
        
        # === PUBLISH QUATERNION SEPARATELY ===
        quat_msg = QuaternionStamped()
        quat_msg.header = processed_imu.header
        quat_msg.quaternion = processed_imu.orientation
        
        self.quat_pub.publish(quat_msg)
        
        # Store for filtering
        self.last_orientation = processed_imu.orientation


def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
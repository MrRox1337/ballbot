#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_filter_arg = DeclareLaunchArgument(
        'use_filter',
        default_value='true',
        description='Enable complementary filter for additional smoothing'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_filter = LaunchConfiguration('use_filter')
    
    # Package paths
    pkg_ballbot_imu = FindPackageShare('ballbot_imu')
    
    # Config file
    config_file = PathJoinSubstitution([
        pkg_ballbot_imu,
        'config',
        'imu_config.yaml'
    ])
    
    # Gazebo IMU bridge
    # This bridges the IMU sensor from Gazebo to ROS
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu_raw@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        name='imu_bridge'
    )
    
    # IMU Processor Node (emulates BNO055)
    imu_processor = Node(
        package='ballbot_imu',
        executable='imu_processor',
        name='imu_processor',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # IMU Filter Node (optional complementary filter)
    imu_filter = Node(
        package='ballbot_imu',
        executable='imu_filter',
        name='imu_filter',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_filter)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_filter_arg,
        imu_bridge,
        imu_processor,
        imu_filter
    ])
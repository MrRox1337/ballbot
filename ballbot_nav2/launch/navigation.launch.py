# Copyright 2025 Aman Mishra
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. Define Paths ---
    pkg_ballbot_nav2 = FindPackageShare('ballbot_nav2')
    pkg_ballbot_slam = FindPackageShare('ballbot_slam')
    pkg_ballbot_control = FindPackageShare('ballbot_control')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    # --- 2. Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    world_type = LaunchConfiguration('world_type')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            pkg_ballbot_slam, 'maps', 'assessment_map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )

    # declare_params_file = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=PathJoinSubstitution([
    #         pkg_ballbot_nav2, 'config', 'ballbot_nav2_params.yaml'
    #     ]),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes'
    # )
    
    declare_world_type = DeclareLaunchArgument(
        'world_type',
        default_value='assessment',
        description='Choose world: "empty" or "assessment"'
    )

    # --- 3. Simulation & Robot Bringup ---
    # This brings up Gazebo, Spawns Robot, Starts Controllers (robot_state_publisher, etc)
    ballbot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ballbot_control, 'launch', 'ballbot_control.launch.py'])
        ),
        launch_arguments={
            'world_type': world_type,
            'use_sim_time': use_sim_time
        }.items()
    )

    # --- 4. Sensor Bridge ---
    # Bridges the Gazebo scan topic to ROS 2
    lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 5. Navigation 2 Bringup (WITH FIX) ---
    # The diff_drive_controller listens on /diff_drive_base_controller/cmd_vel
    # But Nav2 publishes on /cmd_vel by default. We wrap it in a GroupAction to remap it.
    nav2_bringup_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/diff_drive_base_controller/cmd_vel'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    # 'params_file': params_file,
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    # --- 6. RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        # declare_params_file,
        declare_world_type,
        ballbot_control_launch,
        lidar_bridge_node,
        nav2_bringup_launch, 
        rviz_node,
    ])
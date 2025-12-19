"""
Ballbot Bringup Launch File

This is the main entry point for launching the Ballbot robot system.
It provides a unified interface to configure and launch different operational modes.

Launch Arguments:
    world (str): World to load - 'assessment' (default) or 'empty'
    teleop (bool): Launch teleoperation node in a new terminal (default: false)
    slam (bool): Enable SLAM mapping mode (default: false)
    navigation (bool): Enable Nav2 navigation stack (default: false)
    use_sim_time (bool): Use simulation time (default: true)

Usage Examples:
    # Basic simulation with assessment world
    ros2 launch ballbot_bringup bringup.launch.py

    # Simulation with teleoperation
    ros2 launch ballbot_bringup bringup.launch.py teleop:=true

    # SLAM mapping mode
    ros2 launch ballbot_bringup bringup.launch.py slam:=true

    # Autonomous navigation mode
    ros2 launch ballbot_bringup bringup.launch.py navigation:=true

    # Empty world for testing
    ros2 launch ballbot_bringup bringup.launch.py world:=empty teleop:=true

Notes:
    - 'slam' and 'navigation' are mutually exclusive
    - When using 'navigation', set 2D Pose Estimate in RViz before sending goals
    - Run 'ros2 run ballbot_nav2 simple_commander' separately after navigation is ready

Author: Aman Mishra
Email: amanrox97@gmail.com
License: Apache-2.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==========================================================================
    # PACKAGE PATHS
    # ==========================================================================
    pkg_ballbot_bringup = FindPackageShare('ballbot_bringup')
    pkg_ballbot_control = FindPackageShare('ballbot_control')
    pkg_ballbot_description = FindPackageShare('ballbot_description')
    pkg_ballbot_slam = FindPackageShare('ballbot_slam')
    pkg_ballbot_nav2 = FindPackageShare('ballbot_nav2')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    
    # World selection argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='assessment',
        description='World to load: "assessment" (default) or "empty"'
    )
    
    # Teleoperation argument
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='false',
        description='Launch teleoperation node in a new terminal'
    )
    
    # SLAM argument
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Enable SLAM mapping mode'
    )
    
    # Navigation argument
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='false',
        description='Enable Nav2 navigation stack'
    )
    
    # Simulation time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Get launch configurations
    world = LaunchConfiguration('world')
    teleop = LaunchConfiguration('teleop')
    slam = LaunchConfiguration('slam')
    navigation = LaunchConfiguration('navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ==========================================================================
    # WORLD TYPE CONVERSION
    # Convert 'assessment'/'empty' to 'assessment'/'empty' for ballbot_control
    # ==========================================================================
    world_type = PythonExpression([
        "'assessment' if '", world, "' == 'assessment' else 'empty'"
    ])

    # ==========================================================================
    # BASE SIMULATION STACK (Gazebo + Controllers)
    # Always launched - provides the foundation
    # ==========================================================================
    
    # Launch Gazebo + Robot State Publisher + Controllers
    # This uses ballbot_control which internally uses ballbot_gazebo
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ballbot_control, 'launch', 'ballbot_control.launch.py'])
        ),
        launch_arguments={
            'world_type': world_type,
            'use_sim_time': use_sim_time
        }.items(),
        # Only launch base control when NOT using SLAM or Navigation
        # (because SLAM and Navigation launches include their own control)
        condition=UnlessCondition(
            PythonExpression(["'", slam, "' == 'true' or '", navigation, "' == 'true'"])
        )
    )

    # ==========================================================================
    # RVIZ VISUALIZATION
    # Launched for base mode only (SLAM and Nav have their own RViz)
    # ==========================================================================
    rviz_config_file = PathJoinSubstitution(
        [pkg_ballbot_description, 'rviz', 'rviz.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        # Only launch RViz in base mode (SLAM and Nav have their own)
        condition=UnlessCondition(
            PythonExpression(["'", slam, "' == 'true' or '", navigation, "' == 'true'"])
        )
    )

    # ==========================================================================
    # LIDAR BRIDGE
    # Required for base mode visualization (SLAM and Nav include their own)
    # ==========================================================================
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        name='lidar_bridge',
        # Only launch bridge in base mode
        condition=UnlessCondition(
            PythonExpression(["'", slam, "' == 'true' or '", navigation, "' == 'true'"])
        )
    )

    # ==========================================================================
    # TELEOPERATION
    # Launch teleop node in a new terminal when requested
    # Only launches if NOT in SLAM mode (SLAM has its own teleop)
    # ==========================================================================
    teleop_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c',
             'source /opt/ros/jazzy/setup.bash && '
             'source ~/ros2/ballbot_ws/install/setup.bash && '
             'ros2 run ballbot_teleop teleop_flap_node; exec bash'],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", teleop, "' == 'true' and '", slam, "' != 'true'"
            ])
        )
    )
    
    # Delay teleop launch to ensure controllers are ready
    delayed_teleop = TimerAction(
        period=5.0,
        actions=[teleop_node]
    )

    # ==========================================================================
    # SLAM MODE
    # Launches the complete SLAM stack (includes simulation, teleop, and RViz)
    # ==========================================================================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ballbot_slam, 'launch', 'online_async_slam.launch.py'])
        ),
        launch_arguments={
            'world_type': world_type,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(slam)
    )

    # ==========================================================================
    # NAVIGATION MODE
    # Launches Nav2 stack with AMCL localization
    # Note: Does NOT include simple_commander - run that manually
    # ==========================================================================
    
    # Get paths for navigation
    nav2_params_file = PathJoinSubstitution([
        pkg_ballbot_nav2, 'config', 'ballbot_nav2_params.yaml'
    ])
    
    map_file = PathJoinSubstitution([
        pkg_ballbot_slam, 'maps', 'assessment_map.yaml'
    ])
    
    # Navigation launch group - only when navigation:=true
    navigation_group = GroupAction(
        condition=IfCondition(navigation),
        actions=[
            # Launch simulation (Gazebo + Controllers)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_ballbot_control, 'launch', 'ballbot_control.launch.py'
                    ])
                ),
                launch_arguments={
                    'world_type': world_type,
                    'use_sim_time': use_sim_time
                }.items()
            ),
            
            # Lidar bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                name='lidar_bridge_nav'
            ),
            
            # Cmd vel relay (Twist -> TwistStamped)
            Node(
                package='ballbot_nav2',
                executable='cmd_vel_relay',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # AMCL
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Controller server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Planner server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Smoother server
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Behavior server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            
            # Lifecycle manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'map_server',
                        'amcl',
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother'
                    ]
                }],
                output='screen'
            ),
            
            # RViz with Nav2 config
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'
                ])],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            ),
        ]
    )

    # ==========================================================================
    # LOG MESSAGES
    # ==========================================================================
    log_world = LogInfo(
        msg=['Launching Ballbot with world: ', world]
    )
    
    log_mode_base = LogInfo(
        msg=['Mode: Base Simulation'],
        condition=UnlessCondition(
            PythonExpression(["'", slam, "' == 'true' or '", navigation, "' == 'true'"])
        )
    )
    
    log_mode_slam = LogInfo(
        msg=['Mode: SLAM Mapping'],
        condition=IfCondition(slam)
    )
    
    log_mode_nav = LogInfo(
        msg=['Mode: Navigation (Remember to set 2D Pose Estimate in RViz!)'],
        condition=IfCondition(navigation)
    )
    
    log_teleop = LogInfo(
        msg=['Teleoperation: Enabled'],
        condition=IfCondition(teleop)
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    return LaunchDescription([
        # Arguments
        world_arg,
        teleop_arg,
        slam_arg,
        navigation_arg,
        use_sim_time_arg,
        
        # Log messages
        log_world,
        log_mode_base,
        log_mode_slam,
        log_mode_nav,
        log_teleop,
        
        # Base simulation (when not in SLAM or Navigation mode)
        control_launch,
        lidar_bridge,
        rviz_node,
        
        # Teleoperation (when enabled and not in SLAM mode)
        delayed_teleop,
        
        # SLAM mode
        slam_launch,
        
        # Navigation mode
        navigation_group,
    ])
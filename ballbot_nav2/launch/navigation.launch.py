#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_ballbot_nav2 = FindPackageShare('ballbot_nav2')
    pkg_ballbot_slam = FindPackageShare('ballbot_slam')
    pkg_ballbot_control = FindPackageShare('ballbot_control')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_ballbot_nav2, 'config', 'ballbot_nav2_params.yaml'
        ])
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            pkg_ballbot_slam, 'maps', 'assessment_map.yaml'
        ])
    )
    
    # Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ballbot_control, 'launch', 'ballbot_control.launch.py'
            ])
        ),
        launch_arguments={'world_type': 'assessment', 'use_sim_time': use_sim_time}.items()
    )
    
    # Lidar bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Relay
    relay = Node(
        package='ballbot_nav2',
        executable='cmd_vel_relay',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Controller
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Planner
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Smoother
    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Behavior
    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Velocity smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Collision monitor
    # collision_monitor = Node(
    #     package='nav2_collision_monitor',
    #     executable='collision_monitor',
    #     name='collision_monitor',
    #     parameters=[params_file, {'use_sim_time': use_sim_time}],
    #     output='screen'
    # )
    
    # Lifecycle manager - WITHOUT collision_monitor
    lifecycle_manager = Node(
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
                # 'collision_monitor' 
            ]
        }],
        output='screen'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_map,
        sim_launch,
        lidar_bridge,
        relay,
        map_server,
        amcl,
        controller,
        planner,
        smoother,
        behavior,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        # collision_monitor,
        lifecycle_manager,
        rviz
    ])

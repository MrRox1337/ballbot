import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    world_type = LaunchConfiguration('world_type')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ballbot_slam'),
            'config',
            'slam_async_config.yaml'
        ]),
        description='Path to the slam_toolbox parameters file'
    )

    # We default to 'assessment' to match your repo's capabilities
    declare_world_type_cmd = DeclareLaunchArgument(
        'world_type',
        default_value='assessment',
        description='Choose world: "empty" or "assessment"'
    )
    
    # --- 2. Simulation Stack (Control + Gazebo + Robot Description) ---
    # We leverage your existing control launch to handle the heavy lifting.
    # This brings up Gazebo, Spawns the Robot, and Starts Controllers.
    ballbot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ballbot_control'),
                'launch',
                'ballbot_control.launch.py'
            ])
        ),
        launch_arguments={
            'world_type': world_type,
            'use_sim_time': use_sim_time
        }.items()
    )

    # --- 3. Sensor Bridge (Crucial for SLAM) ---
    # We need to bridge the Lidar scan topic from Gazebo (gz.msgs.LaserScan) to ROS 2 (sensor_msgs/msg/LaserScan).
    lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )

    # --- 4. SLAM Toolbox Node ---
    # This brings up the node AND the lifecycle manager automatically
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )
    
    # --- 5. Teleoperation ---
    # Using gnome-terminal for Ubuntu compatibility
    start_teleop_node = Node(
        package='ballbot_teleop',
        executable='teleop_flap_node',
        name='teleop_flap_node',
        output='screen',
        prefix='gnome-terminal --', 
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 6. Visualization (RViz) ---
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ballbot_description'), 'rviz', 'rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 7. Define Launch Description ---
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_world_type_cmd)

    ld.add_action(ballbot_control_launch)
    ld.add_action(lidar_bridge_node)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_teleop_node)
    ld.add_action(rviz_node)

    return ld
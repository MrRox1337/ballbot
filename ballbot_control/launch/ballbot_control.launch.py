import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    
    # 1. Arguments
    world_type_arg = DeclareLaunchArgument(
        'world_type',
        default_value='empty',
        description='Choose world: "empty" or "assessment"'
    )
    
    # --- ADDED: Declare use_sim_time ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    world_type = LaunchConfiguration('world_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. Package Paths
    pkg_ballbot_gazebo = FindPackageShare('ballbot_gazebo')
    pkg_ballbot_control = FindPackageShare('ballbot_control')

    # 3. Launch Simulation (Gazebo + Robot State Publisher + Clock Bridge)
    # --- FIXED: Pass use_sim_time to the included launch ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ballbot_gazebo, 'launch', 'ballbot_gazebo.launch.py'])
        ),
        launch_arguments={
            'world_type': world_type, 
            'use_sim_time': use_sim_time
        }.items()
    )

    # 4. Spawners
    # Note: Controller manager spawners usually auto-detect use_sim_time from the /clock topic
    # if the controller_manager node is running with use_sim_time=True.
    # The gz_ros2_control plugin handles the manager, so it usually syncs automatically.
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller'],
        output='screen'
    )

    flap_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['flap_controller'],
        output='screen'
    )

    # 5. Delay Start
    diff_drive_spawner_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    flap_controller_spawner_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[flap_controller_spawner],
        )
    )

    return LaunchDescription([
        world_type_arg,
        use_sim_time_arg,
        gazebo_launch,
        joint_state_broadcaster_spawner,
        diff_drive_spawner_delayed,
        flap_controller_spawner_delayed
    ])
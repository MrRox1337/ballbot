import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Define Package Paths
    pkg_ballbot_control = FindPackageShare('ballbot_control') 
    pkg_ballbot_description = FindPackageShare('ballbot_description')

    # 2. Define Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    world_type_arg = DeclareLaunchArgument(
        'world_type',
        default_value='empty',
        description='Choose world: "empty" or "assessment"'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_type = LaunchConfiguration('world_type')

    # 3. Include Control Launch (which includes Gazebo)
    # CHANGED: Pass the world_type argument down
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ballbot_control, 'launch', 'ballbot_control.launch.py'])
        ),
        launch_arguments={'world_type': world_type}.items()
    )

    # 4. RViz Node
    rviz_config_file = PathJoinSubstitution(
        [pkg_ballbot_description, 'rviz', 'rviz.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_type_arg,
        control_launch,
        rviz_node
    ])
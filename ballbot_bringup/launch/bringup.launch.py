import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Define Package Paths
    # CHANGE: Point to ballbot_control instead of ballbot_gazebo
    pkg_ballbot_control = FindPackageShare('ballbot_control') 
    pkg_ballbot_description = FindPackageShare('ballbot_description')

    # 2. Define Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. Include Control Launch (which includes Gazebo)
    # CHANGE: This now launches Gazebo AND spawns the controllers
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ballbot_control, 'launch', 'ballbot_control.launch.py'])
        )
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
        control_launch,
        rviz_node
    ])
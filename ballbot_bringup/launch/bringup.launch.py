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
    pkg_ballbot_gazebo = FindPackageShare('ballbot_gazebo')
    pkg_ballbot_description = FindPackageShare('ballbot_description')

    # 2. Define Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. Include Gazebo Launch
    # This launches: Gazebo, Spawning, Robot State Publisher, and the Clock Bridge
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ballbot_gazebo, 'launch', 'ballbot_gazebo.launch.py'])
        )
    )

    # 4. Joint State Bridge
    # Bridges the joint states from Gazebo (gz.msgs.Model) to ROS 2 (sensor_msgs/msg/JointState)
    # The topic format is /world/<world_name>/model/<model_name>/joint_state
    # We assume the world is 'empty' and the model is 'ballbot' as defined in ballbot_gazebo
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        output='screen',
        arguments=[
            '/world/empty/model/ballbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/empty/model/ballbot/joint_state', '/joint_states')
        ]
    )

    # 5. RViz Node
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
        gazebo_launch,
        joint_state_bridge,
        rviz_node
    ])
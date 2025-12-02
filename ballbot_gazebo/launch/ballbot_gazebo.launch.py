import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Path Configuration
    ballbot_description_path = get_package_share_directory('ballbot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the Xacro file
    xacro_file = os.path.join(ballbot_description_path, 'urdf', 'ballbot.urdf.xacro')

    # 2. Process the Xacro file
    # Run xacro to generate the URDF string
    robot_description_config = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # 3. Nodes and Launch Files

    # A. Robot State Publisher
    # Publishes static transforms and the model description
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # B. Gazebo Harmonic Launch
    # Uses the 'gz_sim.launch.py' provided by ros_gz_sim
    # '-r' runs the simulation immediately, 'empty.sdf' loads a void world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # C. Spawn Entity
    # In Jazzy/Harmonic, we use the 'create' executable from ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # Read entity XML from this topic
            '-name', 'ballbot',
            '-z', '0.2' # Spawn higher to avoid floor clipping
        ],
        output='screen'
    )

    # D. ROS-Gazebo Bridge
    # Essential for Jazzy! This bridges the Gazebo clock to ROS 2 /clock
    # so that 'use_sim_time' works correctly.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 4. Return Launch Description
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
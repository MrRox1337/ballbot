import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Declare Arguments
    world_type_arg = DeclareLaunchArgument(
        'world_type',
        default_value='empty',
        description='Choose world: "empty" or "assessment"'
    )
    
    # --- ADDED: Declare use_sim_time argument ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    world_type = LaunchConfiguration('world_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. Path Configuration
    pkg_ballbot_description = FindPackageShare('ballbot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_assessment_world = FindPackageShare('assessment_world')

    # Path to the Xacro file
    xacro_file = PathJoinSubstitution([pkg_ballbot_description, 'urdf', 'ballbot.urdf.xacro'])

    # 3. Process the Xacro file
    robot_description_config = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # 4. Dynamic World Selection Logic
    assessment_world_path = PathJoinSubstitution([
        pkg_assessment_world, 'worlds', 'assessment.sdf'
    ])

    gz_args = PythonExpression([
        "'-r empty.sdf' if '", world_type, "' == 'empty' else ",
        "'-r --render-engine ogre2 ' + '", assessment_world_path, "'"
    ])

    # 5. Nodes and Launch Files

    # A. Robot State Publisher
    # --- FIXED: Added use_sim_time parameter ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # B. Gazebo Harmonic Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # C. Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'ballbot',
            '-z', '0.2',
            '-x', '0.0',
            '-y', '-0.2'
        ],
        output='screen'
    )

    # D. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        world_type_arg,
        use_sim_time_arg, # Added to LD
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
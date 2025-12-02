import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# Import ParameterValue for explicit string conversion
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # The name of the URDF file (assuming you use xacro)
    urdf_file_name = 'ballbot.urdf.xacro'
    
    # 2. Find the share directory for the 'ballbot_description' package
    ballbot_description_pkg_path = get_package_share_directory('ballbot_description')
    
    # Construct the full path to the Xacro file
    urdf_path = PathJoinSubstitution([
        ballbot_description_pkg_path,
        'urdf',
        urdf_file_name
    ])

    # 3. Define the robot description parameter using the 'Command' substitution.
    robot_description_command = Command(['xacro', ' ', urdf_path])
    
    # Wrap the command substitution in ParameterValue to ensure it is treated as a string.
    robot_description_content = ParameterValue(robot_description_command, value_type=str)
    
    # 4. Robot State Publisher Node
    # This node reads the URDF from the 'robot_description' parameter and publishes
    # the robot's transforms to the /tf topic.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            # Pass the dynamically generated URDF content as the parameter value
            {'robot_description': robot_description_content}
        ]
    )

    # 5. Joint State Publisher Node (Non-GUI)
    # This node publishes the initial joint states for visualization purposes.
    # It reads the URDF and sets all non-fixed joints to 0.0 by default.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 6. RViz2 Node
    # Launch RViz to visualize the robot model.
    rviz_config_file = PathJoinSubstitution([
        ballbot_description_pkg_path,
        'rviz',
        'rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )


    # Return the launch description with all actions
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node, # Added the non-GUI JSp here
        rviz_node,
    ])
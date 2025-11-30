import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro

def generate_launch_description():
 
    # Launch Arguments
    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    pkg_name = 'ausrabot_description' # Ensure this matches your package name

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(pkg_name), 'urdf', 'ausrabot_full.urdf.xacro'] # Correct path to your xacro
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] # Pass use_sim_time
    )

    # Gazebo Sim Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_sim.launch.py'])]
        ),
        # Pass gz_args and use empty.sdf world
         launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])],
    )

    # Gazebo Spawn Entity Node
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', # Use topic method like demos
                   '-name', 'ausrabot',          # Robot name in Gazebo
                   '-allow_renaming', 'true',
                   '-z', '0.1'],                 # Initial height
    )


    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity_node,
    ])
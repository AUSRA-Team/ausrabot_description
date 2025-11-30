import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('ausrabot_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'ausrabot_full.urdf.xacro')
    # Process the xacro file to generate the URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Define the path to the RViz config file
    rviz_config_path = os.path.join(pkg_share, 'config', 'ausrabot.rviz')


    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Define the joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Update the RViz2 node to use the config file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] # This is the new line
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
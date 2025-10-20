import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # NEW: Set the GZ_SIM_RESOURCE_PATH environment variable
    # This is crucial for Gazebo to find your model's meshes
    pkg_share = get_package_share_directory('ausrabot_description')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + os.pathsep + os.path.join(pkg_share, '..')
    else:
        gz_resource_path = os.path.join(pkg_share, '..')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # Path to the robot's xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'ausrabot.xacro')
    
    # Process the xacro file to generate the URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Get the path to the controller configuration YAML file
    controller_config = os.path.join(pkg_share, 'config', 'ausrabot_controller.yaml')


    # Start robot_state_publisher
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
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', 'sirius',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.55'
        ]
    )

    # Spawn the Omnidirectional Drive Controller
    spawn_omni_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omnidirectional_controller', '-c', '/controller_manager', '-p', controller_config],
        output='screen'
    )

    # Spawn the Joint State Broadcaster controller
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )


    return LaunchDescription([
        # Add the new environment variable action to the launch description
        set_gz_resource_path,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo,
        spawn_entity_node,
        spawn_omni_controller,
        spawn_joint_state_broadcaster,
    ])

    # Get the path to the controller configuration YAML file
    controller_config = os.path.join(pkg_share, 'config', 'sirius_controller.yaml')


    # Spawn the Joint State Broadcaster controller
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn the Omnidirectional Drive Controller
    spawn_omni_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omnidirectional_controller', '-c', '/controller_manager', '-p', controller_config],
        output='screen'
    )
    
    # Ensure controllers are only loaded after the robot is spawned
    delay_spawners_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[spawn_joint_state_broadcaster, spawn_omni_controller],
        )
    )

    # Return the list of launch actions
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity_node,
        delay_spawners_after_spawn,
        set_gz_resource_path,
    ])
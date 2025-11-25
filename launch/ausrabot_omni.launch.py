import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    pkg_name = 'ausrabot_description' # Ensure this matches your package name
    pkg_share = get_package_share_directory(pkg_name)


    # Set the GZ_SIM_RESOURCE_PATH environment variable
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + os.pathsep + os.path.join(pkg_share, '..')
    else:
        gz_resource_path = os.path.join(pkg_share, '..')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(pkg_name), 'urdf', 'ausrabot.urdf.xacro'] # Correct path to your xacro
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(pkg_name),
            'config',
            'ausrabot_controller.yaml',
        ]
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
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
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', # Use topic method like demos
                   '-name', 'ausrabot',          # Robot name in Gazebo
                   '-allow_renaming', 'true',
                   '-z', '0.1'],                 # Initial height
    )

    # Spawner node for Joint State Broadcaster
    # Note: No '-p' or '--param-file' needed because URDF's Gazebo plugin loads params
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            ],

    )

    # Joint Group Velocity Controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_velocity_controller', '--param-file', robot_controllers],
    )

    # YOUR Custom Omni Driver
    omni_wheels_controller_spawner= Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        output='screen',
        parameters=[robot_controllers, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom')
        ]
    )

    # Clock Bridge (Essential for use_sim_time)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', # Clock
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' # LIDAR
            ],
        output='screen'
    )

    # --- Event Handlers for Sequential Spawning ---
    # Spawn Joint State Broadcaster after Gazebo entity is spawned
    spawn_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    spawn_vel_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    # Spawn Wheel Controllers after Joint State Broadcaster is spawned
    spawn_omni_wheel_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_wheels_controller_spawner],
        )
    )
    # ---------------------------------------------

    return LaunchDescription([
        DeclareLaunchArgument( # Declare use_sim_time argument
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        set_gz_resource_path,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        spawn_broadcaster_event,       # Event Handler 1
        spawn_vel_event,
        spawn_omni_wheel_controllers_event, # Event Handler 2
    ])
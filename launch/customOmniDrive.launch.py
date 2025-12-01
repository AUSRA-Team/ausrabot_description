import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    pkg_name = 'ausrabot_description' 
    pkg_share = get_package_share_directory(pkg_name)

    # --- Gazebo Resource Path Setup ---
    # We must set BOTH GZ_SIM and IGN_GAZEBO paths to support different Gazebo versions
    # Your error log shows 'ign gazebo', so IGN_GAZEBO_RESOURCE_PATH is the critical one.
    
    # 1. Calculate path
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(pkg_share, '..')
    elif 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + os.pathsep + os.path.join(pkg_share, '..')
    else:
        gz_resource_path = os.path.join(pkg_share, '..')

    # 2. Set Environment Variables
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path
    )

    # --- URDF & Config ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(pkg_name), 'urdf', 'ausrabot.urdf.xacro']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    
    robot_controllers_config = PathJoinSubstitution(
        [FindPackageShare(pkg_name), 'config', 'ausrabot_controller.yaml']
    )

    # --- Nodes ---

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 2. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])],
    )

    # 3. Spawn Robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'ausrabot',
                   '-allow_renaming', 'true',
                   '-z', '0.1'],
    )

    # 4. Spawners
    
    # A. Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', robot_controllers_config],
    )

    # B. Joint Group Velocity Controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_velocity_controller', '--param-file', robot_controllers_config],
    )

    # 5. YOUR Custom Omni Driver
    custom_omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        output='screen',
        parameters=[robot_controllers_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom')
        ]
    )

    # 6. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    # --- Dependencies / Events ---
    
    spawn_jsb_event = RegisterEventHandler(
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

    start_python_driver_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velocity_controller_spawner,
            on_exit=[custom_omni_driver],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        # set_ign_resource_path, # Added IGN path
        set_gz_resource_path,  # Kept GZ path for compatibility
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        spawn_jsb_event,
        spawn_vel_event,
        start_python_driver_event,
    ])
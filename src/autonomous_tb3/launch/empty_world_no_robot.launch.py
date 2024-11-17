from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to your custom maze world
    maze_world_path = FindPackageShare('autonomous_tb3').find('autonomous_tb3') + '/worlds/multi_maze.world'

    # Include Gazebo world
    maze_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gzserver.launch.py']),
        launch_arguments={'world': maze_world_path}.items(),
    )

    # Include Gazebo client (GUI)
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gzclient.launch.py']),
    )

    # Spawn TurtleBots
    spawn_turtlebot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot1',
            '-file', FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo') + '/models/turtlebot3_burger/model.sdf',
            '-x', '0', '-y', '0.5', '-z', '0.1',
            '-robot_namespace', '/tb3_6'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    spawn_turtlebot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot2',
            '-file', FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo') + '/models/turtlebot3_burger/model.sdf',
            '-x', '0', '-y', '3.5', '-z', '0.1',
            '-robot_namespace', '/tb3_8'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    spawn_turtlebot3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo') + '/models/turtlebot3_burger/model.sdf',
            '-x', '3', '-y', '0', '-z', '0.1',
            '-robot_namespace', '/tb3_9'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    spawn_turtlebot4 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot4',
            '-file', FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo') + '/models/turtlebot3_burger/model.sdf',
            '-x', '2.8', '-y', '3', '-z', '0.1',
            '-robot_namespace', '/tb3_10'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static transform publishers for each TurtleBot
    static_transform_turtlebot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_scan'],
        namespace='/tb3_6',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    static_transform_turtlebot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_scan'],
        namespace='/tb3_8',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    static_transform_turtlebot3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_scan'],
        namespace='/tb3_9',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    static_transform_turtlebot4 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_scan'],
        namespace='/tb3_10',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        maze_world_launch,
        gzclient_launch,
        spawn_turtlebot1,
        spawn_turtlebot2,
        spawn_turtlebot3,
        spawn_turtlebot4,
        static_transform_turtlebot1,
        static_transform_turtlebot2,
        static_transform_turtlebot3,
        static_transform_turtlebot4
    ])

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    autonomous_tb3_dir = get_package_share_directory('autonomous_tb3')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # File paths
    world_file = os.path.join(autonomous_tb3_dir, 'worlds', 'multi_maze.world')
    maze_model = os.path.join(autonomous_tb3_dir, 'world', 'maze', 'model.sdf')
    map_file = os.path.join(autonomous_tb3_dir, 'maps', 'multi_world.yaml')
    params_file = os.path.join(autonomous_tb3_dir, 'config', 'tb3_nav_params.yaml')
    rviz_config_file = os.path.join(autonomous_tb3_dir, 'rviz', 'tb3_nav.rviz')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-5.2')
    y_pose = LaunchConfiguration('y_pose', default='-6.7')

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn turtlebot in Gazebo
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # Spawn maze model
    maze_spawner = Node(
        package='autonomous_tb3',
        executable='sdf_spawner',
        name='maze_spawner',
        output='screen',
        arguments=[maze_model, "b", "0.0", "0.0"]
    )

    # SLAM Toolbox for mapping
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        )
    )

    # Navigation stack
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # RViz for visualization
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Assemble launch description
    ld = LaunchDescription()

    # Add all launch components
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(maze_spawner)
    ld.add_action(slam_mapping)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld

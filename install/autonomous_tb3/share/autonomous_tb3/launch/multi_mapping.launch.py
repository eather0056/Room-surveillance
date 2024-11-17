from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('autonomous_tb3')
    world_file = os.path.join(package_dir, 'worlds', 'multi_maze.world')
    rviz_config = os.path.join(package_dir, 'rviz', 'cartographer_config.rviz')
    xacro_file = os.path.join(package_dir, 'urdf', 'turtlebot3_burger.urdf.xacro')

    return LaunchDescription([
        # Launch Gazebo with the world file
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_file,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file]),
                'use_sim_time': True
            }]
        ),
        # Spawn TurtleBot3 Entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen',
            parameters=[{'use_sim_time': True}],
            on_exit=[LogInfo(msg="Spawn process completed.")]
        ),
        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', os.path.join(package_dir, 'config'),
                '-configuration_basename', 'tb3_cartographer.lua'
            ]
        ),
        # Cartographer Occupancy Grid
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])

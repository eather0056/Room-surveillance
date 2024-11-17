from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Configuration directory and file for Cartographer
    configuration_directory = '/opt/ros/humble/share/turtlebot3_cartographer/config'
    configuration_basename = 'turtlebot3_lds_2d.lua'

    # Centralized SLAM Node (Cartographer)
    centralized_slam = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('/scan', '/merged_scan'),  # Merged scan topic from all robots
            ('/odom', '/merged_odom')  # Merged odometry topic from all robots
        ],
    )

    # Laser Scan Merger Node
    laser_scan_merger = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[
            {
                'scan_merger/input_topics': [
                    '/tb3_6/scan',
                    '/tb3_8/scan',
                    '/tb3_9/scan',
                    '/tb3_10/scan'
                ],
                'scan_merger/output_topic': '/merged_scan',
                'scan_merger/minimum_angle': -3.14,
                'scan_merger/maximum_angle': 3.14,
                'scan_merger/minimum_range': 0.12,
                'scan_merger/maximum_range': 3.5
            }
        ]
    )

    # Odometry Merger Node
    odometry_merger = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            {
                'frequency': 30.0,
                'odom0': '/tb3_6/odom',
                'odom1': '/tb3_8/odom',
                'odom2': '/tb3_9/odom',
                'odom3': '/tb3_10/odom',
                'odom_topic': '/merged_odom',
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        centralized_slam,
        laser_scan_merger,
        odometry_merger,
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to slam_toolbox launch file
    slam_toolbox_path = FindPackageShare('slam_toolbox').find('slam_toolbox')

    # Include SLAM for each robot
    slam_tb3_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_path, '/launch/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': 'tb3_6'
        }.items()
    )

    slam_tb3_8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_path, '/launch/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': 'tb3_8'
        }.items()
    )

    slam_tb3_9 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_path, '/launch/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': 'tb3_9'
        }.items()
    )

    slam_tb3_10 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_path, '/launch/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': 'tb3_10'
        }.items()
    )

    return LaunchDescription([
        slam_tb3_6,
        slam_tb3_8,
        slam_tb3_9,
        slam_tb3_10,
    ])

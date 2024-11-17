from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths to parameter files for each robot
    tb3_6_param_path = FindPackageShare('autonomous_tb3').find('autonomous_tb3') + '/config/tb3_6_nav2_params.yaml'
    tb3_8_param_path = FindPackageShare('autonomous_tb3').find('autonomous_tb3') + '/config/tb3_8_nav2_params.yaml'
    tb3_9_param_path = FindPackageShare('autonomous_tb3').find('autonomous_tb3') + '/config/tb3_9_nav2_params.yaml'
    tb3_10_param_path = FindPackageShare('autonomous_tb3').find('autonomous_tb3') + '/config/tb3_10_nav2_params.yaml'

    # Include navigation launch for each robot
    nav2_tb3_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('autonomous_tb3'), '/launch/navigation_launch.py']),
        launch_arguments={
            'namespace': 'tb3_6',
            'use_sim_time': 'true',
            'params_file': tb3_6_param_path
        }.items()
    )

    nav2_tb3_8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('autonomous_tb3'), '/launch/navigation_launch.py']),
        launch_arguments={
            'namespace': 'tb3_8',
            'use_sim_time': 'true',
            'params_file': tb3_8_param_path
        }.items()
    )

    nav2_tb3_9 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('autonomous_tb3'), '/launch/navigation_launch.py']),
        launch_arguments={
            'namespace': 'tb3_9',
            'use_sim_time': 'true',
            'params_file': tb3_9_param_path
        }.items()
    )

    nav2_tb3_10 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('autonomous_tb3'), '/launch/navigation_launch.py']),
        launch_arguments={
            'namespace': 'tb3_10',
            'use_sim_time': 'true',
            'params_file': tb3_10_param_path
        }.items()
    )

    return LaunchDescription([
        nav2_tb3_6,
        nav2_tb3_8,
        nav2_tb3_9,
        nav2_tb3_10,
    ])

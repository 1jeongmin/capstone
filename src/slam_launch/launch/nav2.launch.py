import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_slam_launch = get_package_share_directory('slam_launch')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_file = LaunchConfiguration('nav2_params_file')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_slam_launch, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(nav2_bringup)
    return ld

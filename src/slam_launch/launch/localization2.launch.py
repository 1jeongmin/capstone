import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    pkg_slam_launch = get_package_share_directory('slam_launch')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("slam_launch"),   # 내 패키지
            'config',
            'mapper_params_localization.yaml'             # 내가 만든 localization용 yaml
        ),
        description='Full path to the slam_toolbox localization params file'
    )

    serialized_map = os.path.join( os.path.expanduser('~'), 'maps', 'map_5142040')

    lidar_node = Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='sllidar_node',
    parameters=[{
        'channel_type': 'serial',
        'serial_port': '/dev/rplidar',
        'serial_baudrate': 115200,
        'frame_id': 'laser',
        'inverted': False,
        'angle_compensate': True,
        'scan_mode': 'Sensitivity',
    }],
    output='screen'
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[os.path.join(
            pkg_slam_launch,
            'config', 'lidar_filter.yaml'
        )],
        output='screen'
    )
    
    localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'map_file_name': serialized_map}
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(lidar_node)
    ld.add_action(laser_filter_node)
    ld.add_action(localization_node)
    return ld
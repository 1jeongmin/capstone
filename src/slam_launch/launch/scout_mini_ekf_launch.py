import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # EKF 파라미터 파일 경로를 런치 인자로 받을 수 있게 함
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    declare_ekf_params_file_cmd = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(
            get_package_share_directory('slam_launch'),
            'config',
            'scout_mini_ekf.yaml'          # 네가 만든 yaml
        ),
        description='Full path to the robot_localization EKF parameter file'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_params_file,
                    {'use_sim_time': False}] # gazebo에서 쓸때는 True로 바꾸기
    )

    ld = LaunchDescription()
    ld.add_action(declare_ekf_params_file_cmd)
    ld.add_action(ekf_node)
    return ld

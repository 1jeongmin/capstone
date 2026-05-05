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
    pkg_slam_launch = get_package_share_directory('slam_launch')

    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            pkg_slam_launch,
            'config',
            'mapper_params_localization.yaml'
        ),
        description='Full path to the slam_toolbox localization params file'
    )

    # 설치된 패키지 경로 기준으로 맵 파일 지정 (CMakeLists.txt에서 maps/ 설치 필요)
    serialized_map = os.path.join(pkg_slam_launch, 'maps', 'map')

    # robot_description을 scout_description xacro로 생성
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare('scout_description'), 'urdf', 'scout_mini/scout_mini.xacro']
            ),
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content,
        }]
    )

    # 바퀴 joint_states 발행 (RViz RobotModel 시각화용)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
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

    # EKF 노드: wheel odometry(/odom) 융합 → odom→base_link TF 발행
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_launch, 'launch', 'scout_mini_ekf_launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(lidar_node)
    ld.add_action(ekf_launch)
    ld.add_action(localization_node)
    return ld

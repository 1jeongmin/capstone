# launch/scout_mini_delivery_mission.launch.py
"""
5층 한 바퀴 배달/회수 미션 통합 런치.

수행 순서:
  START 출발 → 524호(wp1) 정지 → 로봇팔 배달 → 재출발
            → 521호(wp2) 정지 → 로봇팔 회수 → 재출발
            → (OA 노드가 정적/동적 장애물 자동 회피) → GOAL 도착 정지

실행되는 노드(주행 측, scout_mini_tools):
  - scout_mini_e_stop                : LiDAR 전방 박스 비상정지 (/scout_mini/e_stop)
  - scout_mini_aeb                   : 전방 거리 기반 감속 (/scout_mini/aeb_scale)
  - scout_mini_obstacle_avoidance    : 좌/우 회피 조향 (/scout_mini/oa_steer)
  - sm_path_follower_test4           : Pure Pursuit 경로 추종 (/cmd_vel)
  - mission_manager                  : 정지 지점 판정 + 팔 트리거 오케스트레이터

선행 조건(이 런치는 포함하지 않음 — 최종주행명령어 순서대로 먼저 실행):
  CAN → robot_state_publisher → joint_state_publisher → scout_base
      → odom_covariance_fixer → EKF+IMU → localization(map→base_link TF)
  ※ localization 후 RViz 2D Pose Estimate 로 초기 위치를 잡은 뒤 실행할 것.

로봇팔 노드(elevator-button-robot):
  기본은 별도 터미널에서 실행(매니퓰레이터 HW / RealSense / YOLO 의존).
    ros2 launch open_manipulator_x_bringup hardware.launch.py
    ros2 launch realsense2_camera rs_launch.py ...
    python3 ~/elevator-button-robot/nodes/real_robot/arm_delivery.py
    python3 ~/elevator-button-robot/nodes/real_robot/arm_recover.py
  편의를 위해 arm:=true 로 주면 이 런치에서 arm_delivery/arm_recover 도 함께 실행.

사용법:
  ros2 launch scout_mini_tools scout_mini_delivery_mission.launch.py
  ros2 launch scout_mini_tools scout_mini_delivery_mission.launch.py arm:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 경로 추종에 사용할 waypoint 파일 (scout_mini_autodrive 와 동일 기본값)
WAYPOINT_FILE = os.path.expanduser(
    '~/ros2_ws/src/scout_mini_tools/waypoint/waypoints_map_5floor.yaml'
)

# 로봇팔 노드 경로 (elevator-button-robot)
ARM_DIR = os.path.expanduser('~/elevator-button-robot/nodes/real_robot')


def generate_launch_description():
    arm = LaunchConfiguration('arm')

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='false',
            description='true 면 arm_delivery / arm_recover 도 이 런치에서 실행. '
                        '기본은 별도 터미널 실행.',
        ),

        # 1) E-STOP
        Node(
            package='scout_mini_tools',
            executable='scout_mini_e_stop',
            name='scout_mini_e_stop',
            output='screen',
        ),

        # 2) AEB
        Node(
            package='scout_mini_tools',
            executable='scout_mini_aeb',
            name='scout_mini_aeb',
            output='screen',
        ),

        # 3) Obstacle Avoidance (정적/동적 장애물 회피 조향)
        Node(
            package='scout_mini_tools',
            executable='scout_mini_obstacle_avoidance',
            name='scout_mini_obstacle_avoidance',
            output='screen',
        ),

        # 4) Path Follower (/mission/pause 로 정지/재개)
        Node(
            package='scout_mini_tools',
            executable='sm_path_follower_test4',
            name='sm_path_follower_test4',
            output='screen',
            parameters=[{
                'waypoint_file': WAYPOINT_FILE,
            }],
        ),

        # 5) Mission Manager (정지 지점 판정 + 팔 트리거)
        #    좌표 기본값은 노드 안에 박혀 있음(524호/521호/goal).
        #    필요하면 아래 parameters 로 덮어쓸 수 있음.
        Node(
            package='scout_mini_tools',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'wp1_x': 1.6055,   'wp1_y': 19.7243,   # 523호
                'wp2_x': -3.15392, 'wp2_y': 19.8737,   # 521호
                'goal_x': 2.86,    'goal_y': 4.52,     # 목적지(경로 끝점)
                'wp1_capture': 1.6,
                'wp2_capture': 1.6,
                'approach_hysteresis': 0.15,
                'goal_tol': 0.4,
            }],
        ),

        # 6) (선택) 로봇팔 노드 — arm:=true 일 때만
        ExecuteProcess(
            cmd=['python3', os.path.join(ARM_DIR, 'arm_delivery.py')],
            name='arm_delivery',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(arm),
        ),
        ExecuteProcess(
            cmd=['python3', os.path.join(ARM_DIR, 'arm_recover.py')],
            name='arm_recover',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(arm),
        ),
    ])

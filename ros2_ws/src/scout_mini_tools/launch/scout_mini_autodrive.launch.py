# launch/scout_mini_autodrive.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) E-STOP 노드
        Node(
            package='scout_mini_tools',
            executable='scout_mini_e_stop',   # setup.py에 등록한 이름
            name='scout_mini_e_stop',
            output='screen'
        ),

        # 2) AEB 노드
        Node(
            package='scout_mini_tools',
            executable='scout_mini_aeb',
            name='scout_mini_aeb',
            output='screen',
            # 필요한 경우 AEB 파라미터 여기서도 설정 가능
            # parameters=[{
            #     'fov_deg': 20.0,
            #     'stop_distance': 0.6,
            #     'slow_distance': 1.2,
            # }]
        ),

        Node(
            package='scout_mini_tools',
            executable='scout_mini_obstacle_avoidance',
            name='scout_mini_obstacle_avoidance',
            output='screen',
        ),

        # 3) Path Follower 노드
        Node(
            package='scout_mini_tools',
            executable='sm_path_follower_test4',
            name='sm_path_follower_test4',
            output='screen',
            parameters=[{
                'waypoint_file': '/home/wjdals/ros2_ws/src/scout_mini_tools/waypoint/waypoints_map_5floor.yaml',
            }]
        ),
    ])
####################################아래는 follower용############################################################
# launch/scout_mini_platoon_follower.launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # 1) E-STOP 노드 (follower 차용)
#         Node(
#             package='scout_mini_tools',
#             executable='scout_mini_e_stop',
#             name='scout_mini_e_stop',
#             output='screen',
#         ),

#         # 2) AEB 노드 (follower 차용)
#         Node(
#             package='scout_mini_tools',
#             executable='scout_mini_aeb',
#             name='scout_mini_aeb',
#             output='screen',
#         ),

#         # ✅ 3) follower 전용 Path Follower (플래투닝용)
#         Node(
#             package='scout_mini_tools',
#             executable='sm_platoon_follower_from_leader',
#             name='sm_platoon_follower',
#             output='screen',
#             # 필요하면 여기서 파라미터 조정
#             # parameters=[{
#             #     'leader_waypoint_topic': '/leader/waypoint',
#             #     'desired_gap': 1.0,
#             #     'linear_speed': 0.5,
#             # }]
#         ),
#     ])


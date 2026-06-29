# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 이 패키지(shared dir) 기준 경로
    bringup_dir = get_package_share_directory('scout_mini_nav2')

    # === Launch Arguments ===
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1) 맵 경로 (네 패키지 구조에 맞춤)
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'floor5_test2.yaml'),
        description='Full path to map yaml file to load',
    )

    # 2) AMCL 파라미터 경로 (scout_mini_nav2/params/amcl_params.yaml)
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'amcl_params.yaml'),
        description='Full path to the ROS2 parameters file for localization (AMCL)',
    )

    # 3) 시뮬 타임
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Gazebo 시뮬이면 true 로 실행
        description='Use simulation (Gazebo) clock if true',
    )

    # TF remap & lifecycle 대상 노드
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = ['map_server', 'amcl']

    # === 1) Map Server 노드 ===
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time},
        ],
        remappings=remappings,
    )

    # === 2) AMCL 노드 ===
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=remappings,
    )

    # === 3) Lifecycle Manager ===
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'use_sim_time': use_sim_time,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])

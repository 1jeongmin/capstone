# 완성된 맵 위에서 SLAM을 끄고 자율주행 하고 싶을 때
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/user/maps/floor5_test2.yaml'}]
        )
    ])

# 다른 방법) 홈 디렉토리 자동으로 가져오기 (조금 더 깔끔하며 다른 사람들이 그대로 가져다 이용해도 문제 x)
# 완성된 맵 위에서 SLAM을 끄고 자율주행 하고 싶을 때
#from launch import LaunchDescription
#from launch_ros.actions import Node

#def generate_launch_description():
#    return LaunchDescription([
#        Node(
#            package='nav2_map_server',
#            executable='map_server',
#            name='map_server',
#            output='screen',
#            parameters=[{'yaml_filename': '/home/maps/floor5_test2.yaml'}]
#        )
#    ])


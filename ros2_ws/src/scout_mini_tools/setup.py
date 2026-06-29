from glob import glob
from setuptools import find_packages, setup

package_name = 'scout_mini_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', [
        'launch/scout_mini_autodrive.launch.py',
        'launch/scout_mini_delivery_mission.launch.py',
        ]),

        ('share/' + package_name + '/waypoint',
            glob('waypoint/*.yaml') + glob('waypoint/*.csv')),

        ('share/' + package_name + '/rviz',
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sm_waypoint_recorder = scout_mini_tools.sm_waypoint_recorder:main',
            'path_follower = scout_mini_tools.path_follower:main',
            'sm_path_follower = scout_mini_tools.sm_path_follower:main',
            'sm_path_follower2 = scout_mini_tools.sm_path_follower2:main',
            'sm_path_follower_test = scout_mini_tools.sm_path_follower_test:main',
            'sm_path_follower_original = scout_mini_tools.sm_path_follower_original:main',
            'sm_path_follower_test2 = scout_mini_tools.sm_path_follower_test2:main',
            'scout_mini_e_stop = scout_mini_tools.scout_mini_e_stop:main',
            'scout_mini_aeb = scout_mini_tools.scout_mini_aeb:main',
            'sm_path_follower_test3 = scout_mini_tools.sm_path_follower_test3:main',
            'sm_path_follower_test4 = scout_mini_tools.sm_path_follower_test4:main',
            'scout_mini_obstacle_avoidance = scout_mini_tools.scout_mini_obstacle_avoidance:main',
            'waypoint_node = scout_mini_tools.waypoint_node:main',
            'sm_platoon_follower_from_leader = scout_mini_tools.sm_platoon_follower_from_leader:main',
            'set_initial_pose = scout_mini_tools.set_initial_pose:main',
            'odom_covariance_fixer = scout_mini_tools.odom_covariance_fixer:main',
            'mission_manager = scout_mini_tools.mission_manager:main',
        ],
    },
)

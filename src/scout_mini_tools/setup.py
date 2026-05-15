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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='hjm3423016@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'sm_waypoint_recorder = scout_mini_tools.sm_waypoint_recorder:main',
        'sm_path_follower_test = scout_mini_tools.sm_path_follower_test:main',
    ],
    },
)

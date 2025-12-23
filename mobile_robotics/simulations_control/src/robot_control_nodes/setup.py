from setuptools import find_packages, setup

package_name = 'robot_control_nodes'

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
    maintainer='hackbrian',
    maintainer_email='hackbrian@todo.todo',
    description='Robot control nodes for ArUco tracking, LiDAR obstacle avoidance, and velocity mixing',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tracking_aruco = robot_control_nodes.tracking_aruco:main',
            'lidar_stop = robot_control_nodes.lidar_stop:main',
            'mixer_node = robot_control_nodes.mixer_node:main',
        ],
    },
)

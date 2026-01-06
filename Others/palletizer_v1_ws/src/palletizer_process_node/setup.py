from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'palletizer_process_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hackbrian',
    maintainer_email='user@example.com',
    description='Intelligent palletizing process for box handling with ArUco detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'palletizer_state_machine = palletizer_process_node.palletizer_state_machine:main',
            'navigation_node = palletizer_process_node.navigation_node:main',
            'aruco_tracker_node = palletizer_process_node.aruco_tracker_node:main',
            'box_approach_node = palletizer_process_node.box_approach_node:main',
            'wall_approach_node = palletizer_process_node.wall_approach_node:main',
        ],
    },
)

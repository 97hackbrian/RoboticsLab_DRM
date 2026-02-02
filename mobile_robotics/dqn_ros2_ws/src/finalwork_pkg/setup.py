from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'finalwork_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hackbrian',
    maintainer_email='hackbrian@example.com',
    description='DQN Navigation System for ROS2 with Stage Simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_node = finalwork_pkg.train_node:main',
            'test_node = finalwork_pkg.test_node:main',
        ],
    },
)

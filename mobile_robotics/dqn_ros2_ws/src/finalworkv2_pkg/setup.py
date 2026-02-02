from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'finalworkv2_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hackbrian',
    maintainer_email='hackbrian@example.com',
    description='DQN Navigation for differential robot using Stage simulator with sklearn',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_node = finalworkv2_pkg.train_node:main',
        ],
    },
)

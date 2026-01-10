from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'potential_nav_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brayan',
    maintainer_email='menachocarlos5@gmail.com',
    description='Potential Field Navigation for TurtleBot3',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'potential_field_node = potential_nav_nodes.potential_field_node:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ejercicio1_pkg'

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
    maintainer='Student',
    maintainer_email='student@example.com',
    description='DQN Navigation Agent for Stage Simulator using sklearn MLPRegressor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_node = ejercicio1_pkg.train_node:main',
            'reset_stage = ejercicio1_pkg.reset_stage:main',
        ],
    },
)

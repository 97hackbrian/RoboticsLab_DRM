from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kinematics_mh12'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Carpeta config (si existe)
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Carpeta rviz (si existe)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hackbrian',
    maintainer_email='brayandurantoconas@gmail.com',
    description='kinematics: For the mh12 robot by Yaskawa',
    license='GPL-3.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'forward = kinematics_mh12.fw_kinematics_mh12:main',
            'inverse = kinematics_mh12.inv_kinematics_mh12:main',
        ],
    },
)

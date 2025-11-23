from setuptools import find_packages, setup

package_name = 'forward_kinematics'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_pub=forward_kinematics.joints_pub:main',
            'joint_sub=forward_kinematics.joints_sub:main',
            'fwd_rr=forward_kinematics.fwd_rr:main',
            'fwd_kr6=forward_kinematics.fwd_kr6:main',
            'fwd_ur5=forward_kinematics.fwd_ur5:main',
            'example_poe=forward_kinematics.poe_rrr_ros:main',
        ],
    },
)

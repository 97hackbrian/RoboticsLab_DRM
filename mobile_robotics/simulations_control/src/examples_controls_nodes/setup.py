from setuptools import find_packages, setup

package_name = 'examples_controls_nodes'

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
    maintainer='carlos',
    maintainer_email='menachocarlos5@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "control = control_node.move_base:main",
            "get_odom = control_node.odom_callback:main",
            "img_point = control_node.perception:main",
            "aruco_pose = control_node.aruco_pose:main",
            "tf_sub = control_node.sub_tf:main",
            "teleop = control_node.teleop_robot:main",
            "evasor = control_node.evasor:main"
        ],
    },
)

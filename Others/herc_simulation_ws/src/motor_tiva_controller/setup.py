from setuptools import find_packages, setup

package_name = 'motor_tiva_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeker',
    maintainer_email='seeker@todo.todo',
    description='Controlador ROS2 para motores Tiva suscrito a /cmd_vel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = motor_tiva_controller.motor_node:main',
            'motor_joystick_controller=motor_tiva_controller.motor_joystick_controller:main',

        ],
    },
)


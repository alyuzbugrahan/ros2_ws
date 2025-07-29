from setuptools import setup
import os
from glob import glob

package_name = 'stepper_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Dual stepper motor control package for Jetson Nano with keyboard interface',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stepper_node = stepper_control.stepper_node:main',
            'keyboard_controller = stepper_control.keyboard_controller:main',
            'motor_driver = stepper_control.motor_driver:main',
            'safety_monitor = stepper_control.safety_monitor:main',
            'test_node = stepper_control.test_node:main',
        ],
    },
    scripts=[
        'scripts/stepper_node',
        'scripts/keyboard_controller', 
        'scripts/motor_driver',
        'scripts/safety_monitor',
    ],
) 
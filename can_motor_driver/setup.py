from setuptools import setup
import os
from glob import glob

package_name = 'can_motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can', 'rclpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='ROS 2 node for MKS CAN servo motor control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_motor_node = can_motor_driver.can_motor_node:main',
        ],
    },
)

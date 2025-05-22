from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'robotcatch_robotinterface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-linux-arm',
    maintainer_email='david.seyser@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_interface = robotcatch_robotinterface.abb_egm_interface:main',
            'robot_controll = robotcatch_robotinterface.abb_joint_control:main',
        ],
    },
)

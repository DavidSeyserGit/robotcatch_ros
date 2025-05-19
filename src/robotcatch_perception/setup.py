import os
from glob import glob
from setuptools import setup

package_name = 'robotcatch_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line is crucial for installing the resource directory
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david', # Or your name
    maintainer_email='david@todo.todo', # Or your email
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'camera_interface = robotcatch_perception.camera_interface:main'
        ],
    },
)

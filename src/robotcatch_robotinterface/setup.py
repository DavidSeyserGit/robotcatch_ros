from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotcatch_robotinterface'

# --------------------------------------------------------------------
# Walk the 'urdf/' folder and collect *every* file under it
urdf_data = []
for root, _, files in os.walk('urdf'):
    # skip hidden dirs
    if os.path.basename(root).startswith('.'):
        continue
    src_paths = [
        os.path.join(root, f)
        for f in files
        if not f.startswith('.')    # skip .gitkeep etc
    ]
    if not src_paths:
        continue
    # preserve the same relative structure under share/<pkg>/
    # e.g. root == 'urdf/abb_irb120_support/meshes'
    target_dir = os.path.join('share', package_name, root)
    urdf_data.append((target_dir, src_paths))
# --------------------------------------------------------------------

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # for ros2 pkg prefix ...
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        # your package.xml
        (f'share/{package_name}', ['package.xml']),
        # your launch files
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ] + urdf_data,  # <-- tack on all the URDF+mesh files here
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-linux-arm',
    maintainer_email='david.seyser@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_stream = '
              'robotcatch_robotinterface.abb_joint_stream:main',
            'robot_control = '
              'robotcatch_robotinterface.abb_joint_control:main',
        ],
    },
)

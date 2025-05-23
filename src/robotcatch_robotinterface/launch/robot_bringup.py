from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robotcatch_robotinterface')
    urdf_file = os.path.join(
      pkg_share, 'urdf', 'abb_irb120_support', 'urdf', 'abbIrb120.urdf'
    )

    return LaunchDescription([
        # 1) real‐robot joint publisher
        Node(
          package='robotcatch_robotinterface',
          executable='robot_stream',
          name='abb_joint_publisher',
          output='screen',
          # if robot_stream publishes on "abb_robot/joint_states", remap it:
          remappings=[('abb_robot/joint_states', 'joint_states')],
          parameters=[{
            'control_frequency': 1000,
            'joint_names': [
              'joint_1','joint_2','joint_3',
              'joint_4','joint_5','joint_6'
            ],
          }],
        ),

        # 3) drive TFs out of the URDF
        Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{ 'robot_description': open(urdf_file).read() }],
        ),

        # 4) RViz
        Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          output='screen',
          # you can pass in a pre‐made rviz config if you like:
          arguments=['-d', os.path.join(pkg_share, 'config', 'digital_twin.rviz')]
        ),
    ])

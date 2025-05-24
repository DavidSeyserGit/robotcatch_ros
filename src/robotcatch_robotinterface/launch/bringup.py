# abb_robot_driver/launch/abb_egm.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the ABB EGM node
    abb_egm_node = Node(
        package='robotcatch_robotinterface',
        executable='robot_interface',
        name='abb_joint_publisher',
        output='screen',
        parameters=[
            {
                'control_frequency': 1000,  # Hz
                'joint_names': [
                    'joint_1', 'joint_2', 'joint_3', 
                    'joint_4', 'joint_5', 'joint_6'
                ],
            }
        ],
        # Define remappings if needed
        remappings=[
            # Add any topic remappings here if required
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        abb_egm_node
    ])

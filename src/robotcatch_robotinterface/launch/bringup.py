# abb_robot_driver/launch/abb_egm.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.125.1',
        description='IP address of the ABB robot controller'
    )
    
    egm_port_arg = DeclareLaunchArgument(
        'egm_port',
        default_value='6510',
        description='Port for EGM communication'
    )
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    egm_port = LaunchConfiguration('egm_port')
    
    # Define the ABB EGM node
    abb_egm_node = Node(
        package='robotcatch_robotinterface',
        executable='robot_interface',
        name='abb_robot_egm_node',
        output='screen',
        parameters=[
            {
                'robot_ip': robot_ip,
                'egm_port': egm_port,
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
        robot_ip_arg,
        egm_port_arg,
        abb_egm_node
    ])

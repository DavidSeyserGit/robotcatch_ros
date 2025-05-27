from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    align_depth_arg = DeclareLaunchArgument(
        'align_depth.enable',
        default_value='true',
        description='Enable depth alignment to color'
    )
    
    pointcloud_arg = DeclareLaunchArgument(
        'pointcloud.enable',
        default_value='true',
        description='Enable pointcloud generation'
    )
    
    initial_reset_arg = DeclareLaunchArgument(
        'initial_reset',
        default_value='true',
        description='Reset camera on startup'
    )

    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='camera',
        parameters=[{
            'align_depth.enable': LaunchConfiguration('align_depth.enable'),
            'pointcloud.enable': LaunchConfiguration('pointcloud.enable'),
            'initial_reset': LaunchConfiguration('initial_reset'),
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'color_fps': 60,
            'depth_fps': 60,
        }],
        output='screen'
    )

    # Your custom YOLO ball detection node
    yolo_node = Node(
        package='robotcatch_perception',
        executable='camera_interface',  
        name='camera_interface',
        output='screen',
        parameters=[{
        }]
    )

    return LaunchDescription([
        align_depth_arg,
        pointcloud_arg,
        initial_reset_arg,
        realsense_node,
        yolo_node,
    ])
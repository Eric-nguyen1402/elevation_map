from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elevation_mapping',
            executable='elevation_mapping_node',
            name='elevation_mapping',
            remappings=[('cloud_in', '/camera/depth/color/points')],
            parameters=[{'world_frame_id': 'base_link'}],
            output='screen'
        )
    ])

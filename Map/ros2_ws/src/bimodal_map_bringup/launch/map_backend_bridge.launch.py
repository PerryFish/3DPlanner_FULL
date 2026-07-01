from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_mode = LaunchConfiguration('backend_mode')
    return LaunchDescription([
        DeclareLaunchArgument('backend_mode', default_value='fallback'),
        Node(
            package='bimodal_3d_map_adapter',
            executable='map_backend_bridge_node',
            output='screen',
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/map_backend.yaml',
                {'backend_mode': backend_mode},
            ],
        ),
    ])

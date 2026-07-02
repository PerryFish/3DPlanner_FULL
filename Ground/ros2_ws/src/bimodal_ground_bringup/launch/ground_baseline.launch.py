from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_mode = LaunchConfiguration('planner_mode')
    config_file = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument('planner_mode', default_value='stub'),
        DeclareLaunchArgument('config_file', default_value='/home/nuaa/ZHY/3DPlanner_FULL/Ground/config/ground_explorer.yaml'),
        Node(package='bimodal_ground_explorer', executable='ground_3d_frontier_node', output='screen',
             parameters=[config_file, {'planner_mode': planner_mode}])
    ])

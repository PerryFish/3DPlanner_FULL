from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='bimodal_ground_explorer', executable='ground_3d_frontier_node', output='screen',
             parameters=['/home/nuaa/ZHY/3DPlanner_FULL/Ground/config/ground_explorer.yaml'])
    ])

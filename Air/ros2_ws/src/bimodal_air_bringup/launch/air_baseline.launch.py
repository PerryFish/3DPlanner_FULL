from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='bimodal_air_adapter', executable='air_exploration_stub_node', output='screen',
             parameters=['/home/nuaa/ZHY/3DPlanner_FULL/Air/config/air_adapter.yaml'])
    ])

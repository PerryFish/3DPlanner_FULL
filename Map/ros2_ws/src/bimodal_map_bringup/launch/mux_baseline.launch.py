from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mode_mux_config = '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/mode_mux.yaml'
    return LaunchDescription([
        Node(package='bimodal_mode_mux', executable='bimodal_mode_mux_node', output='screen',
             parameters=[mode_mux_config]),
        Node(package='bimodal_mode_mux', executable='simple_mode_commander_node', output='screen',
             parameters=[mode_mux_config]),
    ])

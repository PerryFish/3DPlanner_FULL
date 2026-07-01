from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='bimodal_virtual_sensors', executable='virtual_sensor_node', output='screen',
             parameters=['/home/nuaa/ZHY/3DPlanner_FULL/Map/config/virtual_sensors.yaml']),
        Node(package='bimodal_3d_map_adapter', executable='fallback_3d_map_adapter_node', output='screen',
             parameters=['/home/nuaa/ZHY/3DPlanner_FULL/Map/config/map_adapter.yaml']),
    ])

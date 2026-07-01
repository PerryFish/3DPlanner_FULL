from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_external_odom = LaunchConfiguration('use_external_odom_for_virtual_sensor')
    virtual_publish_odom = LaunchConfiguration('virtual_sensor_publish_odom')
    backend_mode = LaunchConfiguration('backend_mode')
    sensor_range = LaunchConfiguration('sensor_range')

    return LaunchDescription([
        DeclareLaunchArgument('backend_mode', default_value='octomap_style_voxel'),
        DeclareLaunchArgument('use_external_odom_for_virtual_sensor', default_value='true'),
        DeclareLaunchArgument('virtual_sensor_publish_odom', default_value='false'),
        DeclareLaunchArgument('sensor_range', default_value='5.0'),
        Node(
            package='bimodal_virtual_sensors',
            executable='virtual_sensor_node',
            output='screen',
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/virtual_sensors.yaml',
                {
                    'publish_odom': ParameterValue(virtual_publish_odom, value_type=bool),
                    'use_external_odom': ParameterValue(use_external_odom, value_type=bool),
                    'publish_tf': False,
                    'publish_world_gt_cloud': True,
                    'publish_world_full_cloud': False,
                    'publish_local_sensor_cloud': True,
                    'sensor_range': ParameterValue(sensor_range, value_type=float),
                    'sensor_fov_deg': 160.0,
                },
            ],
        ),
        Node(
            package='bimodal_3d_map_adapter',
            executable='octomap_pointcloud_backend_node',
            output='screen',
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/octomap_backend.yaml',
                {'backend_mode': backend_mode},
            ],
        ),
        Node(
            package='bimodal_e2e_sim_tools',
            executable='visual_tf_guard_node',
            output='screen',
            parameters=[{
                'enable_tf_guard': True,
                'dynamic_tf_publish_hz': 20.0,
                'static_tf_republish_period_sec': 2.0,
            }],
        ),
    ])

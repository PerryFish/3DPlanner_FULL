from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    e2e_log_dir = LaunchConfiguration('e2e_log_dir')
    use_external_odom = LaunchConfiguration('use_external_odom_for_virtual_sensor')
    virtual_publish_odom = LaunchConfiguration('virtual_sensor_publish_odom')
    fallback_accumulate = LaunchConfiguration('fallback_accumulate_map')
    mode_auto_switch = LaunchConfiguration('mode_auto_switch')
    mode_switch_period = LaunchConfiguration('mode_switch_period_sec')
    active_mode_period = LaunchConfiguration('active_mode_publish_period_sec')
    active_path_period = LaunchConfiguration('active_path_republish_period_sec')
    sensor_range = LaunchConfiguration('sensor_range')
    publish_world_gt = LaunchConfiguration('publish_world_gt_cloud')
    publish_local_cloud = LaunchConfiguration('publish_local_sensor_cloud')

    return LaunchDescription([
        DeclareLaunchArgument('e2e_log_dir', default_value='/tmp/bimodal_visual_demo'),
        DeclareLaunchArgument('run_duration_sec', default_value='180'),
        DeclareLaunchArgument('use_external_odom_for_virtual_sensor', default_value='true'),
        DeclareLaunchArgument('virtual_sensor_publish_odom', default_value='false'),
        DeclareLaunchArgument('fallback_accumulate_map', default_value='true'),
        DeclareLaunchArgument('publish_world_gt_cloud', default_value='true'),
        DeclareLaunchArgument('publish_local_sensor_cloud', default_value='true'),
        DeclareLaunchArgument('mode_auto_switch', default_value='true'),
        DeclareLaunchArgument('mode_switch_period_sec', default_value='45.0'),
        DeclareLaunchArgument('active_mode_publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('active_path_republish_period_sec', default_value='1.0'),
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
                    'publish_world_gt_cloud': ParameterValue(publish_world_gt, value_type=bool),
                    'publish_world_full_cloud': False,
                    'publish_local_sensor_cloud': ParameterValue(publish_local_cloud, value_type=bool),
                    'sensor_range': ParameterValue(sensor_range, value_type=float),
                    'sensor_fov_deg': 160.0,
                },
            ],
        ),
        Node(
            package='bimodal_3d_map_adapter',
            executable='fallback_3d_map_adapter_node',
            output='screen',
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/map_adapter.yaml',
                {
                    'accumulate_map': ParameterValue(fallback_accumulate, value_type=bool),
                    'max_coverage_marker_count': 3000,
                },
            ],
        ),
        Node(
            package='bimodal_mode_mux',
            executable='bimodal_mode_mux_node',
            output='screen',
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/mode_mux.yaml',
                {'republish_period_sec': ParameterValue(active_path_period, value_type=float)},
            ],
        ),
        Node(
            package='bimodal_mode_mux',
            executable='simple_mode_commander_node',
            output='screen',
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/mode_mux.yaml',
                {
                    'enable_auto_switch': ParameterValue(mode_auto_switch, value_type=bool),
                    'switch_period_sec': ParameterValue(mode_switch_period, value_type=float),
                    'publish_period_sec': ParameterValue(active_mode_period, value_type=float),
                    'initial_mode': 'AIR',
                },
            ],
        ),
        Node(
            package='bimodal_e2e_sim_tools',
            executable='fake_path_executor_node',
            output='screen',
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
        Node(
            package='bimodal_e2e_sim_tools',
            executable='e2e_metrics_logger_node',
            output='screen',
            parameters=[{'log_dir': e2e_log_dir}],
        ),
    ])

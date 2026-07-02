from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    e2e_log_dir = LaunchConfiguration('e2e_log_dir')
    sensor_input_mode = LaunchConfiguration('sensor_input_mode')
    input_topic = LaunchConfiguration('input_topic')
    enable_synthetic = LaunchConfiguration('enable_synthetic_pointcloud')
    scene_profile = LaunchConfiguration('scene_profile')
    enable_explainability_overlay = LaunchConfiguration('enable_explainability_overlay')
    backend_mode = LaunchConfiguration('backend_mode')
    mode_switch_period = LaunchConfiguration('mode_switch_period_sec')
    active_path_period = LaunchConfiguration('active_path_republish_period_sec')

    virtual_points_enabled = PythonExpression([
        "'", sensor_input_mode, "' == 'virtual'",
    ])
    bridge_enabled = PythonExpression([
        "'", sensor_input_mode, "' != 'virtual'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('e2e_log_dir', default_value='/tmp/bimodal_p1c_sensor_input'),
        DeclareLaunchArgument('sensor_input_mode', default_value='external_pointcloud'),
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('enable_synthetic_pointcloud', default_value='true'),
        DeclareLaunchArgument('scene_profile', default_value='default_sparse'),
        DeclareLaunchArgument('enable_explainability_overlay', default_value='false'),
        DeclareLaunchArgument('backend_mode', default_value='octomap_style_voxel'),
        DeclareLaunchArgument('mode_switch_period_sec', default_value='75.0'),
        DeclareLaunchArgument('active_path_republish_period_sec', default_value='1.0'),
        Node(
            package='bimodal_virtual_sensors',
            executable='virtual_sensor_node',
            output='screen',
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/virtual_sensors.yaml',
                {
                    'publish_points': ParameterValue(virtual_points_enabled, value_type=bool),
                    'publish_odom': False,
                    'use_external_odom': True,
                    'publish_tf': False,
                    'publish_world_gt_cloud': False,
                    'publish_world_full_cloud': False,
                    'publish_local_sensor_cloud': ParameterValue(virtual_points_enabled, value_type=bool),
                    'sensor_range': 5.0,
                    'sensor_fov_deg': 160.0,
                },
            ],
        ),
        Node(
            package='bimodal_3d_map_adapter',
            executable='synthetic_external_pointcloud_publisher_node',
            output='screen',
            condition=IfCondition(enable_synthetic),
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/p1c_sensor_input.yaml',
                {'output_topic': input_topic, 'scene_profile': scene_profile},
            ],
        ),
        Node(
            package='bimodal_3d_map_adapter',
            executable='real_sensor_pointcloud_bridge_node',
            output='screen',
            condition=IfCondition(bridge_enabled),
            parameters=[
                '/home/nuaa/ZHY/3DPlanner_FULL/Map/config/p1c_sensor_input.yaml',
                {
                    'sensor_input_mode': sensor_input_mode,
                    'input_topic': input_topic,
                    'output_topic': '/bimodal/points',
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
                    'enable_auto_switch': True,
                    'switch_period_sec': ParameterValue(mode_switch_period, value_type=float),
                    'publish_period_sec': 1.0,
                    'initial_mode': 'AIR',
                },
            ],
        ),
        Node(package='bimodal_e2e_sim_tools', executable='fake_path_executor_node', output='screen'),
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
            executable='demo_explainability_overlay_node',
            output='screen',
            condition=IfCondition(enable_explainability_overlay),
            parameters=[{'scene_profile': scene_profile}],
        ),
        Node(
            package='bimodal_e2e_sim_tools',
            executable='e2e_metrics_logger_node',
            output='screen',
            parameters=[{'log_dir': e2e_log_dir}],
        ),
    ])

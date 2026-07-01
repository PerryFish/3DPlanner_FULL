from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['rviz2', '-d', '/home/nuaa/ZHY/3DPlanner_FULL/Map/rviz/bimodal_baseline.rviz'], output='screen')
    ])

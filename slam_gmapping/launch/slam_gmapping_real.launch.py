"""真车 gmapping 启动（use_sim_time=false，不依赖 /clock）"""
import os
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("slam_gmapping"),
        "params", "slam_gmapping_real.yaml")
    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[params]),
    ])

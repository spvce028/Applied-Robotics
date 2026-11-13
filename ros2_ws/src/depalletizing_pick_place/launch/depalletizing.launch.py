
"""Launch the depalletizing node with YAML configuration."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('depalletizing_pick_place'),
        'config',
        'depalletizing.yaml'
    )

    return LaunchDescription([
        Node(
            package='depalletizing_pick_place',
            executable='depalletizing_node',
            name='depalletizing_main_node',
            output='screen',
            parameters=[config_file],
        )
    ])

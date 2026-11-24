"""Launch hardware clients or a complete hardware-free simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    """Resolve mode before choosing configuration and optional dependencies."""
    simulation = LaunchConfiguration('simulation').perform(context) == 'true'
    share = get_package_share_directory('depalletizing_pick_place')
    config = LaunchConfiguration('config').perform(context)
    if not config:
        config = os.path.join(
            share, 'config', 'simulation.yaml' if simulation else 'depalletizing.yaml',
        )
    nodes = []
    if simulation:
        nodes.extend([
            Node(
                package='depalletizing_pick_place', executable='fake_ptp_action_server',
                output='screen',
            ),
            Node(
                package='depalletizing_pick_place', executable='fake_gripper_service',
                output='screen',
            ),
        ])
    # Hardware mode deliberately leaves DOBOT bringup to the operator.
    nodes.append(Node(
        package='depalletizing_pick_place', executable='depalletizing_node',
        name='depalletizing_main_node', output='screen', parameters=[config],
    ))
    if LaunchConfiguration('use_rviz').perform(context) == 'true':
        description = get_package_share_directory('dobot_description')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, 'launch', 'display.launch.py'),
            ),
            launch_arguments={
                'DOF': '4', 'tool': 'gripper',
                'gui': 'true' if simulation else 'false',
            }.items(),
        ))
    return nodes


def generate_launch_description():
    """Expose explicit mode, configuration and model-visualization options."""
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('config', default_value='', description='Optional YAML override'),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_nodes),
    ])

"""Depth-to-scan only. Existing driver and TF authorities must already run."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('aurora_depth_obstacle_scan'), 'config', 'diagnostic.yaml'])),
        Node(package='aurora_depth_obstacle_scan', executable='aurora_depth_obstacle_scan',
             name='aurora_depth_obstacle_scan', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])

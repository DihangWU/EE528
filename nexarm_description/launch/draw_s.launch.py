"""Launch the standalone RViz horizontal drawing demo."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('nexarm_description')
    description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'nexarm.urdf.xacro')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('height', default_value='0.0'),
        DeclareLaunchArgument('duration', default_value='16.0'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='nexarm_description', executable='draw_s', output='screen',
             parameters=[{'robot_description': description,
                          'height': ParameterValue(LaunchConfiguration('height'), value_type=float),
                          'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'draw_s.rviz')]),
    ])

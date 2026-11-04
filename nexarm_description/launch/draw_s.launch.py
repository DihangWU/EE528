"""Launch the standalone RViz horizontal drawing demo."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('nexarm_description')
    # Older terminals may have sourced this workspace before the panel was built.
    # Give the new RViz process the installed plugin prefix explicitly.
    try:
        plugin_prefix = Path(get_package_prefix('nexarm_rviz_plugins'))
    except PackageNotFoundError:
        plugin_prefix = Path(share).parent.parent.parent / 'nexarm_rviz_plugins'
    resource = plugin_prefix / 'share/ament_index/resource_index/rviz_common__pluginlib__plugin/nexarm_rviz_plugins'
    if not resource.is_file():
        raise RuntimeError('RViz replay plugin is not installed. Run: '
                           'colcon build --symlink-install --packages-up-to nexarm_description')
    rviz_environment = {
        'AMENT_PREFIX_PATH': str(plugin_prefix) + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', '')}
    description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'nexarm.urdf.xacro')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('height', default_value='-0.10'),
        DeclareLaunchArgument('duration', default_value='50.0'),
        DeclareLaunchArgument('lift', default_value='0.015'),
        DeclareLaunchArgument('error_deg', default_value='0.2'),
        DeclareLaunchArgument('error_seed', default_value='42'),
        DeclareLaunchArgument('mpc_enabled', default_value='false'),
        DeclareLaunchArgument('motor_tau', default_value='0.2'),
        *[DeclareLaunchArgument(f'motor_tau_j{i}', default_value=str(tau))
          for i, tau in enumerate((0.6, 0.5, 0.6, 0.4, 0.0, 0.0), start=1)],
        DeclareLaunchArgument('mpc_horizon', default_value='12'),
        DeclareLaunchArgument('mpc_control_horizon', default_value='8'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='nexarm_description', executable='draw_s', output='screen',
             parameters=[{'robot_description': description,
                          **{f'motor_tau_j{i}': ParameterValue(LaunchConfiguration(f'motor_tau_j{i}'), value_type=float) for i in range(1, 7)},
                          'motor_tau': ParameterValue(LaunchConfiguration('motor_tau'), value_type=float),
                          'mpc_enabled': ParameterValue(LaunchConfiguration('mpc_enabled'), value_type=bool),
                          'mpc_horizon': ParameterValue(LaunchConfiguration('mpc_horizon'), value_type=int),
                          'mpc_control_horizon': ParameterValue(LaunchConfiguration('mpc_control_horizon'), value_type=int),
                          'error_deg': ParameterValue(LaunchConfiguration('error_deg'), value_type=float),
                          'error_seed': ParameterValue(LaunchConfiguration('error_seed'), value_type=int),
                          'lift': ParameterValue(LaunchConfiguration('lift'), value_type=float),
                          'height': ParameterValue(LaunchConfiguration('height'), value_type=float),
                          'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', additional_env=rviz_environment,
             arguments=['-d', os.path.join(share, 'rviz', 'draw_s.rviz')]),
    ])

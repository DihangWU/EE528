"""Launch manual degree input panel, robot TF publisher, and RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('nexarm_description')
    model = os.path.join(share, 'urdf/nexarm.urdf.xacro')
    panel = ExecuteProcess(cmd=['/usr/bin/python3', os.path.join(
        share, 'UI/angle_panel.py'), '--model', model], output='screen')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', model]), value_type=str)}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz/view.rviz')]),
        RegisterEventHandler(OnProcessExit(target_action=panel,
            on_exit=[EmitEvent(event=Shutdown(reason='Angle panel closed'))])),
        panel,
    ])

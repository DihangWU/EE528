"""Gazebo Harmonic joint-controlled drawing demo."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('nexarm_description')
    description = ParameterValue(Command(['xacro ', os.path.join(
        share, 'urdf/nexarm.urdf.xacro'), ' gazebo_demo:=true']), value_type=str)
    spawn = Node(package='ros_gz_sim', executable='create', arguments=[
        '-world', 'drawing', '-name', 'nexarm', '-topic', 'robot_description'], output='screen')
    draw = Node(package='nexarm_description', executable='draw_s', output='screen',
                parameters=[{'robot_description': description, 'gazebo': True,
                             'use_sim_time': True}])
    bridges = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
               '/nexarm/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model']
    bridges += [f'/nexarm/joint{i}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
                for i in range(1, 6)]
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r ' + os.path.join(share, 'worlds/drawing.sdf')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}],
             remappings=[('joint_states', '/nexarm/joint_states')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridges),
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=[TimerAction(period=2.0, actions=[draw])])),
        spawn,
    ])

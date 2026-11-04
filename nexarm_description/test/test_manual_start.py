"""The real ROS writer must stay prepared and still until an explicit start."""
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
import yaml


def test_prepared_pose_waits_for_explicit_start(tmp_path):
    model = Path(__file__).parents[1] / 'urdf/nexarm.urdf.xacro'
    params = tmp_path / 'writer.yaml'
    params.write_text(yaml.safe_dump({'draw_s': {'ros__parameters': {
        'robot_description': model.read_text(), 'error_deg': .2,
        'motor_tau': .2}}}))
    env = dict(os.environ, ROS_DOMAIN_ID='89')
    log_path = tmp_path / 'writer.log'
    context = Context()
    rclpy.init(context=context, domain_id=89)
    node = Node('manual_start_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    states, started, errors, rms, ink_counts = [], [], [], [], []
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: states.append(msg.position), 10)
    node.create_subscription(Bool, '/writing/started',
                             lambda msg: started.append(msg.data), 10)
    node.create_subscription(Float64, '/writing/pen_error_mm',
                             lambda msg: errors.append(msg.data), 10)
    node.create_subscription(Float64, '/writing/pen_rms_mm',
                             lambda msg: rms.append(msg.data), 10)

    def markers(message):
        for marker in message.markers:
            if marker.id == 0:
                ink_counts.append(len(marker.points) if marker.action == 0 else 0)
    node.create_subscription(MarkerArray, '/drawing_markers', markers, 10)
    replay = node.create_client(Trigger, '/restart_writing')
    settings = AsyncParameterClient(node, '/draw_s')

    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'run', 'nexarm_description', 'draw_s', '--ros-args',
             '--params-file', str(params)], env=env, stdout=log, stderr=log,
            start_new_session=True)

        def wait(predicate, timeout=20):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
                assert process.poll() is None, log_path.read_text()
                if predicate():
                    return
            pytest.fail('Timed out waiting for writer: ' + log_path.read_text())

        def complete(future):
            wait(future.done)
            return future.result()

        try:
            wait(lambda: replay.service_is_ready() and settings.services_are_ready()
                 and len(states) >= 30 and len(started) >= 20 and len(ink_counts) >= 20)
            assert not any(started)
            assert not any(ink_counts)
            assert np.allclose(states, states[0], atol=0, rtol=0)
            assert np.all(np.asarray(errors) == 0) and np.all(np.asarray(rms) == 0)
            # Editing valid pending settings must also leave the robot idle.
            response = complete(settings.set_parameters_atomically([
                Parameter('pid_enabled', value=True)]))
            assert response.result.successful
            old_count = len(states)
            wait(lambda: len(states) >= old_count + 15)
            assert not any(started) and not any(ink_counts)
            assert np.allclose(states, states[0], atol=0, rtol=0)
            response = complete(replay.call_async(Trigger.Request()))
            assert response.success, response.message
            wait(lambda: any(started) and any(ink_counts))
            assert not np.allclose(states[-1], states[0], atol=1e-6)
            # The same action remains usable for subsequent replay.
            response = complete(replay.call_async(Trigger.Request()))
            assert response.success
        finally:
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)

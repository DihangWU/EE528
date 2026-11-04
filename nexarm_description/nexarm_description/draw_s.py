"""Horizontal S drawing demo with a virtual pen and bounded inverse kinematics."""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def chain_from_xml(xml):
    """Read the actual model dimensions and limits."""
    root = ET.fromstring(xml)
    names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint']
    chain = []
    for name in names:
        joint = root.find(f"joint[@name='{name}']")
        origin = joint.find('origin')
        transform = np.eye(4)
        transform[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
        transform[:3, :3] = Rotation.from_euler(
            'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        axis = joint.find('axis')
        axis = np.fromstring(axis.get('xyz'), sep=' ')
        limit = joint.find('limit')
        chain.append((transform, axis, joint.get('type'), limit))
    return chain


def forward(chain, q):
    """Return gripper pose, with joint5 held at zero."""
    pose = np.eye(4)
    for i, (origin, axis, kind, _) in enumerate(chain):
        pose = pose @ origin
        if kind == 'revolute' and i < 4:
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * q[i]).as_matrix()
            pose = pose @ rotation
    return pose


def plan(chain, height=0.0, duration=16.0):
    """Preflight the complete path before publishing any motion."""
    if not math.isfinite(height) or not math.isfinite(duration) or duration < 4:
        raise ValueError('height must be finite and duration must be at least 4 seconds')
    limits = ([float(j[3].get('lower')) for j in chain[:4]],
              [float(j[3].get('upper')) for j in chain[:4]])
    count = math.ceil(duration * 30) + 1
    t = np.linspace(0, 1, count)
    # Two tangent 270-degree arcs form rounded, almost closed S lobes.
    # Smooth time scaling preserves the common tangent at the join.
    u = t * t * (3 - 2 * t)
    radius = 0.035
    upper = u <= 0.5
    angle = np.where(upper, 3 * np.pi * u,
                     np.pi / 2 - 3 * np.pi * (u - 0.5))
    targets = np.column_stack((0.27 + radius * np.cos(angle),
                               np.where(upper, radius, -radius)
                               + radius * np.sin(angle),
                               np.full(count, height)))
    q = np.array([0., 0.5, -0.4, 1.47])
    # Pen mounts along gripper -Z, perpendicular to its forward +X axis.
    # Zero total pitch keeps the gripper horizontal and the pen vertical.
    positions, tips = [], []
    for target in targets:
        def residual(angles):
            pose = forward(chain, angles)
            tip = pose[:3, 3] + -pose[:3, 2] * 0.10
            return np.r_[tip - target, 0.1 * (sum(angles[1:]))]
        result = least_squares(residual, q, bounds=limits, gtol=1e-11,
                               ftol=1e-11, xtol=1e-11)
        q = result.x
        pose = forward(chain, q)
        tip = pose[:3, 3] + -pose[:3, 2] * 0.10
        if np.linalg.norm(tip - target) > 0.0005 or np.linalg.norm(residual(q)) > 0.0005:
            raise ValueError(f'Unreachable drawing point: {target}')
        positions.append(q.copy())
        tips.append(tip)
    positions = np.array(positions)
    if np.max(np.abs(np.diff(positions, axis=0))) * 30 > 3.14:
        raise ValueError('Trajectory exceeds joint velocity limits; increase duration')
    return positions, np.array(tips)


def main(args=None):
    """Publish joint states and visible pen strokes for RViz."""
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init(args=args)
    node = Node('draw_s')
    node.declare_parameter('robot_description', '')
    node.declare_parameter('gazebo', False)
    gazebo = node.get_parameter('gazebo').value
    node.declare_parameter('height', 0.0)
    node.declare_parameter('duration', 16.0)
    chain = chain_from_xml(node.get_parameter('robot_description').value)
    try:
        positions, tips = plan(chain, node.get_parameter('height').value,
                               node.get_parameter('duration').value)
    except ValueError as error:
        node.get_logger().error(str(error))
        node.destroy_node()
        rclpy.shutdown()
        return
    joints = node.create_publisher(JointState, '/joint_states', 10)
    markers = node.create_publisher(MarkerArray, '/drawing_markers', 10)
    commands = [node.create_publisher(Float64, f'/nexarm/joint{i}/cmd_pos', 10)
                for i in range(1, 6)] if gazebo else []
    start = None

    def point(p):
        return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))

    def tick():
        nonlocal start
        now = node.get_clock().now()
        if now.nanoseconds == 0:
            return
        if start is None:
            start = now
        elapsed = max(0., (now - start).nanoseconds / 1e9 - (5. if gazebo else 0.))
        index = min(int(elapsed * 30), len(positions) - 1)
        q = positions[index]
        stamp = node.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                      'gripper_base_joint', 'left_jaw_joint', 'right_jaw_joint']
        state.position = [*map(float, q), 0., 0., 0., 0.]
        if gazebo:
            for publisher, value in zip(commands, [*q, 0.]):
                publisher.publish(Float64(data=float(value)))
        else:
            joints.publish(state)
        stroke = Marker()
        stroke.header.frame_id = 'base_link'
        stroke.header.stamp = stamp
        stroke.ns = 'drawing'
        stroke.id = 0
        stroke.type = Marker.LINE_STRIP
        stroke.pose.orientation.w = 1.
        stroke.scale.x = 0.003
        stroke.color.r, stroke.color.g, stroke.color.b, stroke.color.a = (0.1, 0.8, 1., 1.)
        stroke.points = [point(p) for p in tips[:index + 1]]
        pen = Marker()
        pen.header = stroke.header
        pen.ns = 'drawing'
        pen.id = 1
        pen.type = Marker.LINE_LIST
        pen.pose.orientation.w = 1.
        pen.scale.x = 0.006
        pen.color.r, pen.color.g, pen.color.b, pen.color.a = (1., 0.6, 0.1, 1.)
        pen.points = [point(forward(chain, q)[:3, 3]), point(tips[index])]
        markers.publish(MarkerArray(markers=[stroke, pen]))

    node.create_timer(1 / 30, tick)
    node.get_logger().info('Drawing S on the horizontal plane; holds at the end.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

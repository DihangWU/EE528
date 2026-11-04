"""Write ee528 with separate strokes, pen lifts and bounded inverse kinematics."""
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


def text_strokes():
    """Return ee528 strokes on the ground, in readable top-view order."""
    def bezier(a, b, c, d):
        t = np.linspace(0, 1, 17)[:, None]
        return ((1 - t) ** 3 * np.array(a)
                + 3 * (1 - t) ** 2 * t * np.array(b)
                + 3 * (1 - t) * t ** 2 * np.array(c) + t ** 3 * np.array(d))

    # Lowercase e: crossbar followed by an open, rounded loop.
    angle = np.linspace(0, 1.90 * np.pi, 73)
    e = [np.vstack(([[0, .5]], np.column_stack((.5 + .5 * np.cos(angle),
                                                .5 + .5 * np.sin(angle)))))]
    two = [np.vstack((
        bezier((0, .78), (.1, 1.12), (.95, 1.12), (1, .78)),
        bezier((1, .78), (1, .55), (.2, .25), (0, 0))[1:], [[1, 0]]))]
    five = [np.vstack(([[1, 1], [0, 1], [0, .55]],
                        bezier((0, .55), (.4, .7), (1, .65), (1, .3))[1:],
                        bezier((1, .3), (1, -.08), (.15, -.08), (0, .15))[1:]))]
    # One continuous figure-eight, crossing the waist without lifting.
    theta = np.linspace(0, 2 * np.pi, 97)
    eight = [np.column_stack((.5 + .5 * np.sin(2 * theta),
                              .5 + .5 * np.sin(theta)))]
    strokes = []
    for index, glyph in enumerate([e, e, five, two, eight]):
        for stroke in glyph:
            uv = np.asarray(stroke)
            strokes.append(np.column_stack((.25 + .045 * uv[:, 1],
                                             .085 - .036 * index - .025 * uv[:, 0])))
    return strokes


def writing_targets(height=0.0, duration=50.0, lift=0.015):
    """Sample pen-down strokes and vertical lift/travel/lower transitions."""
    if (not all(math.isfinite(v) for v in [height, duration, lift])
            or duration < 4 or lift <= 0):
        raise ValueError('Finite height/duration/lift required; duration >= 4, lift > 0')
    segments = []
    strokes = text_strokes()
    current = np.r_[strokes[0][0], height + lift]
    initial = current.copy()
    for stroke in strokes:
        above = np.r_[stroke[0], height + lift]
        down = np.r_[stroke[0], height]
        for target, ink in [(above, False), (down, False)]:
            if np.linalg.norm(target - current) > 1e-9:
                segments.append((current.copy(), target, ink))
            current = target
        for xy in stroke[1:]:
            target = np.r_[xy, height]
            segments.append((current.copy(), target, True))
            current = target
        target = current + [0, 0, lift]
        segments.append((current.copy(), target, False))
        current = target
    # Every segment stops smoothly, including corners and pen lifts.
    weights = np.array([np.linalg.norm(b - a) for a, b, _ in segments])
    counts = np.maximum(3, np.ceil(duration * 30 * weights / weights.sum()).astype(int))
    targets, ink = [initial], [False]
    for (a, b, drawing), count in zip(segments, counts):
        t = np.linspace(0, 1, count + 1)[1:]
        u = t * t * (3 - 2 * t)
        targets.extend(a + u[:, None] * (b - a))
        ink.extend([drawing] * count)
    return np.array(targets), np.array(ink, dtype=bool)


def plan(chain, height=0.0, duration=50.0, lift=0.015):
    """Preflight every writing and pen-up point before publishing motion."""
    targets, ink = writing_targets(height, duration, lift)
    limits = ([float(j[3].get('lower')) for j in chain[:4]],
              [float(j[3].get('upper')) for j in chain[:4]])
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
    return positions, np.array(tips), ink


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
    node.declare_parameter('duration', 50.0)
    node.declare_parameter('lift', 0.015)
    chain = chain_from_xml(node.get_parameter('robot_description').value)
    try:
        positions, tips, ink = plan(chain, node.get_parameter('height').value,
                               node.get_parameter('duration').value,
                               node.get_parameter('lift').value)
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
        stroke.type = Marker.LINE_LIST
        stroke.pose.orientation.w = 1.
        stroke.scale.x = 0.003
        stroke.color.r, stroke.color.g, stroke.color.b, stroke.color.a = (0.1, 0.8, 1., 1.)
        stroke.points = [point(p) for i in range(1, index + 1) if ink[i]
                         for p in (tips[i - 1], tips[i])]
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
    node.get_logger().info('Writing ee528 with pen lifts; holds above the final stroke.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

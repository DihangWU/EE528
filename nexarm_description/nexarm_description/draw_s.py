"""Write ee528 and WU Dihang with separate strokes, pen lifts and bounded inverse kinematics."""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


PEN_LENGTH = 0.125
PEN_FORWARD = 0.04
GRIPPER_HOLD = (0.038561 - 0.0131701054 - 0.003) / 0.05


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
    """Return gripper pose; an omitted joint5 angle defaults to zero."""
    pose = np.eye(4)
    for i, (origin, axis, kind, _) in enumerate(chain):
        pose = pose @ origin
        if kind == 'revolute' and i < len(q):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * q[i]).as_matrix()
            pose = pose @ rotation
    return pose


def text_strokes():
    """Return two lines on the ground, in readable top-view order."""
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
    # Shift both lines forward to keep the lower row within the J2 limit.
    strokes = []
    for index, glyph in enumerate([e, e, five, two, eight]):
        for stroke in glyph:
            uv = np.asarray(stroke)
            strokes.append(np.column_stack((.355 + .045 * uv[:, 1],
                                             .085 - .036 * index - .025 * uv[:, 0])))
    # Second line: capitals W/U/D, followed by lowercase i/h/a/n/g.
    # Coordinates use a common baseline, with a descender on the lowercase g.
    def oval(width, height):
        angle = np.linspace(0, 2 * np.pi, 41)
        return np.column_stack((width / 2 * (1 + np.cos(angle)),
                                height / 2 * (1 + np.sin(angle))))

    w = [np.array([[0, 1], [.25, 0], [.6, .65], [.95, 0], [1.2, 1]])]
    u = [np.vstack(([[0, 1], [0, .3]],
                    bezier((0, .3), (0, -.1), (1, -.1), (1, .3))[1:], [[1, 1]]))]
    d = [np.array([[0, 0], [0, 1]]),
         np.vstack((bezier((0, 1), (1.25, 1), (1.25, 0), (0, 0))))]
    i = [np.array([[.125, .65], [.125, 0]]),
         oval(.12, .10) + [.065, .82]]
    h = [np.array([[0, 1], [0, 0]]),
         np.vstack((bezier((0, .42), (.15, .85), (.8, .82), (.8, .38)), [[.8, 0]]))]
    a = [oval(.8, .68), np.array([[.8, .68], [.8, 0]])]
    n = [np.vstack(([[0, 0], [0, .68], [0, .42]],
                    bezier((0, .42), (.15, .85), (.8, .82), (.8, .38))[1:], [[.8, 0]]))]
    g = [oval(.8, .68),
         np.vstack(([[.8, .68], [.8, -.15]],
                    bezier((.8, -.15), (.8, -.48), (.1, -.48), (0, -.22))[1:]))]
    line = []
    cursor = 0.0
    for glyph, width in [(w, 1.2), (u, 1), (None, .5), (d, 1),
                         (i, .25), (h, .8), (a, .8), (n, .8), (g, .8)]:
        if glyph is not None:
            line.extend(np.asarray(stroke) + [cursor, 0] for stroke in glyph)
        cursor += width + .25
    # Match the actual ink width and horizontal alignment of the first line.
    first = np.vstack(strokes)
    second = np.vstack(line)
    left, right = np.min(first[:, 1]), np.max(first[:, 1])
    umin, umax = np.min(second[:, 0]), np.max(second[:, 0])
    for uv in line:
        strokes.append(np.column_stack((.305 + .035 * uv[:, 1],
                                        right - (uv[:, 0] - umin) / (umax - umin)
                                        * (right - left))))
    return strokes


def writing_targets(height=-0.10, duration=50.0, lift=0.015):
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
    targets = np.array(targets)
    return targets, np.array(ink, dtype=bool)


def plan(chain, height=-0.10, duration=50.0, lift=0.015):
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
            tip = pose[:3, 3] + pose[:3, 0] * PEN_FORWARD - pose[:3, 2] * PEN_LENGTH
            return np.r_[tip - target, 0.1 * (sum(angles[1:]))]
        result = least_squares(residual, q, bounds=limits, gtol=1e-11,
                               ftol=1e-11, xtol=1e-11)
        q = result.x
        pose = forward(chain, q)
        tip = pose[:3, 3] + pose[:3, 0] * PEN_FORWARD - pose[:3, 2] * PEN_LENGTH
        if np.linalg.norm(tip - target) > 0.0005 or np.linalg.norm(residual(q)) > 0.0005:
            raise ValueError(f'Unreachable drawing point: {target}')
        positions.append(q.copy())
        tips.append(tip)
    positions = np.array(positions)
    if np.max(np.abs(np.diff(positions, axis=0))) * 30 > 3.14:
        raise ValueError('Trajectory exceeds joint velocity limits; increase duration')
    return positions, np.array(tips), ink


def simulate_arrival_errors(chain, positions, error_deg=0.2, seed=42):
    """Apply bounded, time-varying motor position error to J1--J5.

    error_deg bounds the additive angle error before joint-limit clipping.
    A fixed seed reproduces the same independent per-motor disturbance.
    No controller, compensator, or physical contact model is included.
    """
    if not math.isfinite(error_deg) or error_deg < 0:
        raise ValueError('error_deg must be finite and nonnegative')
    if not isinstance(seed, int) or seed < 0:
        raise ValueError('error_seed must be a nonnegative integer')
    requested = np.column_stack((positions, np.zeros(len(positions))))
    actual = requested.copy()
    if error_deg:
        rng = np.random.default_rng(seed)
        amplitude = math.radians(error_deg)
        innovations = rng.uniform(-amplitude, amplitude, requested.shape)
        error = np.zeros(5)
        lower = np.array([float(j[3].get('lower')) for j in chain[:5]])
        upper = np.array([float(j[3].get('upper')) for j in chain[:5]])
        for i, innovation in enumerate(innovations):
            # Correlated arrival jitter, bounded by error_deg at every sample.
            error = 0.35 * error + 0.65 * innovation
            actual[i] = np.clip(requested[i] + error, lower, upper)
    tips = []
    for q in actual:
        pose = forward(chain, q)
        tips.append(pose[:3, 3] + pose[:3, 0] * PEN_FORWARD - pose[:3, 2] * PEN_LENGTH)
    return actual, np.array(tips)


def main(args=None):
    """Publish joint states and visible pen strokes for RViz."""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    import signal
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from std_srvs.srv import Trigger
    from rcl_interfaces.msg import SetParametersResult
    from nexarm_mpc.simulation import JointTrackingSimulation
    from nexarm_mpc.linear_incremental import resolve_motor_taus
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init(args=args)
    node = Node('draw_s')
    node.declare_parameter('robot_description', '')
    node.declare_parameter('gazebo', False)
    gazebo = node.get_parameter('gazebo').value
    node.declare_parameter('height', -0.10)
    node.declare_parameter('duration', 50.0)
    node.declare_parameter('lift', 0.015)
    node.declare_parameter('error_deg', 0.0)
    node.declare_parameter('error_seed', 42)
    node.declare_parameter('mpc_enabled', False)
    node.declare_parameter('mpc_horizon', 12)
    node.declare_parameter('mpc_control_horizon', 8)
    node.declare_parameter('motor_tau', 0.0)
    tau_names = [f'motor_tau_j{i}' for i in range(1, 7)]
    for name in tau_names:
        node.declare_parameter(name, -1.0)
    chain = chain_from_xml(node.get_parameter('robot_description').value)
    try:
        positions, tips, ink = plan(chain, node.get_parameter('height').value,
                               node.get_parameter('duration').value,
                               node.get_parameter('lift').value)
        reference_tips = tips.copy()
        reference_positions = np.column_stack((positions, np.zeros(len(positions)),
                                                np.full(len(positions), GRIPPER_HOLD)))
        root = ET.fromstring(node.get_parameter('robot_description').value)
        servo_limits = [j[3] for j in chain[:5]] + [
            root.find("joint[@name='gripper_base_joint']/limit")]

        def make_simulation():
            enabled = node.get_parameter('mpc_enabled').value
            if gazebo and enabled:
                raise ValueError('MPC currently supports simulated RViz feedback only.')
            return JointTrackingSimulation(
                reference_positions,
                [float(limit.get('lower')) for limit in servo_limits],
                [float(limit.get('upper')) for limit in servo_limits],
                [float(limit.get('velocity')) for limit in servo_limits],
                error_deg=node.get_parameter('error_deg').value,
                seed=node.get_parameter('error_seed').value, mpc_enabled=enabled,
                horizon=node.get_parameter('mpc_horizon').value,
                control_horizon=node.get_parameter('mpc_control_horizon').value,
                motor_tau=resolve_motor_taus(node.get_parameter('motor_tau').value,
                    [node.get_parameter(name).value for name in tau_names]), error_joints=5)

        simulation = make_simulation()
        active_error_deg = node.get_parameter('error_deg').value
        mpc_enabled = node.get_parameter('mpc_enabled').value
        tips = np.full_like(reference_tips, np.nan)
    except (KeyboardInterrupt, ExternalShutdownException):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    except ValueError as error:
        node.get_logger().error(str(error))
        node.destroy_node()
        rclpy.shutdown()
        return
    def validate_pending_settings(parameters):
        # ROS parameters are pending settings; the active run uses a snapshot.
        names = ['error_deg', 'error_seed', 'mpc_enabled', 'mpc_horizon',
                 'mpc_control_horizon', 'motor_tau'] + tau_names
        values = {name: node.get_parameter(name).value for name in names}
        values.update({p.name: p.value for p in parameters if p.name in names})
        if not math.isfinite(values['error_deg']) or values['error_deg'] < 0:
            return SetParametersResult(successful=False, reason='error_deg must be finite and >= 0')
        if not math.isfinite(values['motor_tau']) or values['motor_tau'] < 0:
            return SetParametersResult(successful=False, reason='motor_tau must be finite and >= 0')
        try:
            resolve_motor_taus(values['motor_tau'], [values[name] for name in tau_names])
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
        if values['error_seed'] < 0:
            return SetParametersResult(successful=False, reason='error_seed must be >= 0')
        if not 1 <= values['mpc_control_horizon'] <= values['mpc_horizon'] <= 100:
            return SetParametersResult(successful=False, reason='Require 1 <= Nc <= Np <= 100')
        if gazebo and values['mpc_enabled']:
            return SetParametersResult(successful=False, reason='MPC requires RViz simulated feedback')
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(validate_pending_settings)
    joints = node.create_publisher(JointState, '/joint_states', 10)
    markers = node.create_publisher(MarkerArray, '/drawing_markers', 1)
    pen_error = node.create_publisher(Float64, '/writing/pen_error_mm', 10)
    pen_rms = node.create_publisher(Float64, '/writing/pen_rms_mm', 10)
    commands = [node.create_publisher(Float64, f'/nexarm/joint{i}/cmd_pos', 10)
                for i in range(1, 6)] if gazebo else []
    if gazebo:
        commands.extend(node.create_publisher(Float64, f'/nexarm/{name}/cmd_pos', 10)
                        for name in ['gripper_base_joint', 'left_jaw_joint', 'right_jaw_joint'])
    start = None
    stats_reported = False
    ink_error_squared_sum = 0.0
    ink_error_count = 0

    def point(p):
        return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))

    # Actual stroke vertices are appended only after feedback has been observed.
    stroke_vertices = []
    reference_vertices = [point(p) for i in range(1, len(reference_tips)) if ink[i]
                          for p in (reference_tips[i - 1], reference_tips[i])]

    def tick():
        nonlocal start, stats_reported, ink_error_squared_sum, ink_error_count
        now = node.get_clock().now()
        if now.nanoseconds == 0:
            return
        if start is None:
            start = now
        elapsed = max(0., (now - start).nanoseconds / 1e9 - (5. if gazebo else 0.))
        index = min(int(elapsed * 30), len(reference_positions) - 1)
        # Process every fixed 1/30 s plant step, including when a UI frame is late.
        while simulation.index < index:
            actual = simulation.step()
            sample = simulation.index
            pose = forward(chain, actual)
            tips[sample] = (pose[:3, 3] + PEN_FORWARD * pose[:3, 0]
                            - PEN_LENGTH * pose[:3, 2])
            if ink[sample]:
                difference = tips[sample] - reference_tips[sample]
                ink_error_squared_sum += float(difference @ difference)
                ink_error_count += 1
                stroke_vertices.extend([point(tips[sample - 1]), point(tips[sample])])
        q = simulation.measured
        error_mm = float(np.linalg.norm(tips[index] - reference_tips[index]) * 1000)
        pen_error.publish(Float64(data=error_mm))
        rms_mm = math.sqrt(ink_error_squared_sum / ink_error_count) * 1000 if ink_error_count else 0.
        pen_rms.publish(Float64(data=rms_mm))
        if index == len(reference_positions) - 1 and not stats_reported:
            errors = tips[ink] - reference_tips[ink]
            rms = float(np.sqrt(np.mean(np.sum(errors ** 2, axis=1))) * 1000)
            extra = ''
            if simulation.controller:
                controller = simulation.controller
                extra = (f', QP fallbacks={controller.fallbacks}, '
                         f'solve p99={np.percentile(controller.solve_times, 99)*1000:.3f} ms')
            node.get_logger().info(f'Playback finished: pen RMS={rms:.4f} mm{extra}')
            stats_reported = True
        stamp = node.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                      'gripper_base_joint', 'left_jaw_joint', 'right_jaw_joint']
        state.position = [*map(float, q), -0.05 * float(q[5]), 0.05 * float(q[5])]
        if gazebo:
            for publisher, value in zip(commands, state.position):
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
        stroke.scale.x = 0.0006
        stroke.color.r, stroke.color.g, stroke.color.b, stroke.color.a = (
            (0.1, 0.8, 1., 1.) if mpc_enabled else (1., 0.3, 0.15, 1.))
        stroke.points = stroke_vertices
        if not stroke.points:
            stroke.action = Marker.DELETE
        # Delete the legacy Marker pen: the rigid URDF pen now follows robot TF.
        clear_pen = Marker()
        clear_pen.ns = 'drawing'
        clear_pen.id = 1
        clear_pen.action = Marker.DELETE
        visible = [stroke, clear_pen]
        # Keep the ideal reference visible with or without random arrival error.
        reference = Marker()
        reference.header = stroke.header
        reference.ns = 'drawing'
        reference.id = 2
        reference.type = Marker.LINE_LIST
        reference.pose.orientation.w = 1.
        reference.scale.x = 0.0003
        reference.color.r = reference.color.g = reference.color.b = 0.7
        reference.color.a = 0.85
        reference.points = reference_vertices
        visible.append(reference)
        markers.publish(MarkerArray(markers=visible))

    def restart_writing(request, response):
        nonlocal start, stats_reported, simulation, active_error_deg, mpc_enabled
        nonlocal ink_error_squared_sum, ink_error_count
        # Construct the new run first; a failure must leave the current run intact.
        try:
            candidate = make_simulation()
        except (ValueError, np.linalg.LinAlgError) as error:
            response.success = False
            response.message = str(error)
            return response
        simulation = candidate
        active_error_deg = node.get_parameter('error_deg').value
        mpc_enabled = node.get_parameter('mpc_enabled').value
        start = None
        stats_reported = False
        tips.fill(np.nan)
        stroke_vertices.clear()
        ink_error_squared_sum = 0.0
        ink_error_count = 0
        clear = Marker()
        clear.ns = 'drawing'
        clear.id = 0
        clear.action = Marker.DELETE
        markers.publish(MarkerArray(markers=[clear]))
        response.success = True
        response.message = 'Writing restarted from the first sample.'
        node.get_logger().info(
            f'Restart requested: cleared ink and reset playback; '
            f'active error={active_error_deg} deg, MPC={mpc_enabled}, '
            f'motor taus={simulation.motor_tau.tolist()} s.')
        return response

    node.create_service(Trigger, '/restart_writing', restart_writing)
    node.create_timer(1 / 30, tick)
    error_deg = node.get_parameter('error_deg').value
    node.get_logger().info(
        f'Writing ee528 / WU Dihang; motor arrival error bound={error_deg} deg, '
        f'seed={node.get_parameter("error_seed").value}, '
        f'Linear Constrained Incremental MPC={mpc_enabled}, '
        f'motor taus={simulation.motor_tau.tolist()} s. '
        'Colored stroke follows actual joints (MPC on: cyan, off: orange-red); gray is ideal.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch may forward a second SIGINT while ROS entities are being destroyed.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

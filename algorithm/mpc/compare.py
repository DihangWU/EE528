#!/usr/bin/env python3
"""Compare off/PID/LQR/MPC with identical disturbances on the full writing path."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from nexarm_description.draw_s import GRIPPER_HOLD, PEN_FORWARD, PEN_LENGTH, chain_from_xml, forward, plan
from nexarm_mpc.simulation import JointTrackingSimulation
from nexarm_mpc.linear_incremental import resolve_motor_taus


def contour_rms_mm(actual, reference, ink):
    """XY distance to reference segments of the same stroke, independent of timing."""
    starts = np.flatnonzero(ink & ~np.r_[False, ink[:-1]])
    ends = np.flatnonzero(ink & ~np.r_[ink[1:], False])
    distances = []
    for start, end in zip(starts, ends):
        a = reference[start - 1:end, :2]
        direction = reference[start:end + 1, :2] - a
        delta = actual[start:end + 1, None, :2] - a[None, :, :]
        squared_length = np.sum(direction ** 2, axis=1)
        fraction = np.clip(np.sum(delta * direction, axis=2)
                           / np.maximum(squared_length, 1e-24), 0, 1)
        difference = delta - fraction[:, :, None] * direction
        distances.extend(np.min(np.sum(difference ** 2, axis=2), axis=1))
    return float(np.sqrt(np.mean(distances)) * 1000)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--error-deg', type=float, default=.2)
    parser.add_argument('--motor-tau', type=float, default=.2)
    parser.add_argument('--motor-taus', nargs=6, type=float, default=None, metavar='SECONDS')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', default='algorithm/results/comparison.json')
    parser.add_argument('--modes', nargs='+', choices=['off', 'pid', 'lqr', 'mpc'],
                        default=['off', 'pid', 'lqr', 'mpc'])
    args = parser.parse_args()
    model = Path(get_package_share_directory('nexarm_description')) / 'urdf/nexarm.urdf.xacro'
    xml = xacro.process_file(str(model)).toxml()
    chain = chain_from_xml(xml)
    root = ET.fromstring(xml)
    limits = [j[3] for j in chain[:5]] + [root.find("joint[@name='gripper_base_joint']/limit")]
    taus = resolve_motor_taus(args.motor_tau, args.motor_taus or [-1.] * 6)
    q, ideal, ink = plan(chain)
    reference = np.column_stack((q, np.zeros(len(q)), np.full(len(q), GRIPPER_HOLD)))
    results = {'error_deg': args.error_deg, 'seed': args.seed,
               'motor_tau_s': args.motor_tau, 'motor_taus_s': taus.tolist(),
               'sample_hz': 30, 'samples': len(q), 'evaluation': 'pen-down samples'}
    for mode in args.modes:
        simulation = JointTrackingSimulation(
            reference, [float(limit.get('lower')) for limit in limits],
            [float(limit.get('upper')) for limit in limits],
            [float(limit.get('velocity')) for limit in limits],
            error_deg=args.error_deg, seed=args.seed, control_mode=mode,
            motor_tau=taus, error_joints=5)
        tips = []
        for _ in reference:
            actual = simulation.step()
            pose = forward(chain, actual)
            tips.append(pose[:3, 3] + PEN_FORWARD * pose[:3, 0] - PEN_LENGTH * pose[:3, 2])
        error = np.asarray(tips)[ink] - ideal[ink]
        results[mode] = {
            'pen_xy_contour_rms_mm': contour_rms_mm(np.asarray(tips), ideal, ink),
            'pen_xyz_rms_mm': float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))) * 1000),
            'pen_xy_rms_mm': float(np.sqrt(np.mean(np.sum(error[:, :2] ** 2, axis=1))) * 1000),
            'joint_rms_deg': float(np.rad2deg(np.sqrt(np.mean(
                (simulation.actual[ink, :5] - reference[ink, :5]) ** 2)))),
        }
        if simulation.controller:
            results[mode].update({
                'fallbacks': simulation.controller.fallbacks,
                'active_joint_qps': simulation.controller.active_qps,
                'solve_mean_ms': float(np.mean(simulation.controller.solve_times) * 1000),
                'solve_p99_ms': float(np.percentile(simulation.controller.solve_times, 99) * 1000),
            })
    if 'off' in results and results['off']['pen_xyz_rms_mm'] > 0:
        results['pen_xyz_improvement_percent'] = {
            mode: 100 * (1 - results[mode]['pen_xyz_rms_mm'] / results['off']['pen_xyz_rms_mm'])
            for mode in args.modes if mode != 'off'}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Compare MPC off/on with identical disturbances on the full writing path."""
import argparse
import json
from pathlib import Path

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from nexarm_description.draw_s import PEN_FORWARD, PEN_LENGTH, chain_from_xml, forward, plan
from nexarm_mpc.simulation import JointTrackingSimulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--error-deg', type=float, default=.2)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', default='mpc/results/comparison.json')
    args = parser.parse_args()
    model = Path(get_package_share_directory('nexarm_description')) / 'urdf/nexarm.urdf.xacro'
    chain = chain_from_xml(xacro.process_file(str(model)).toxml())
    q, ideal, ink = plan(chain)
    reference = np.column_stack((q, np.zeros(len(q))))
    results = {'error_deg': args.error_deg, 'seed': args.seed,
               'sample_hz': 30, 'samples': len(q), 'evaluation': 'pen-down samples'}
    for mode in ['off', 'on']:
        simulation = JointTrackingSimulation(
            reference, [float(j[3].get('lower')) for j in chain[:5]],
            [float(j[3].get('upper')) for j in chain[:5]],
            [float(j[3].get('velocity')) for j in chain[:5]],
            error_deg=args.error_deg, seed=args.seed, mpc_enabled=mode == 'on')
        tips = []
        for _ in reference:
            actual = simulation.step()
            pose = forward(chain, actual)
            tips.append(pose[:3, 3] + PEN_FORWARD * pose[:3, 0] - PEN_LENGTH * pose[:3, 2])
        error = np.asarray(tips)[ink] - ideal[ink]
        results[mode] = {
            'pen_xyz_rms_mm': float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))) * 1000),
            'pen_xy_rms_mm': float(np.sqrt(np.mean(np.sum(error[:, :2] ** 2, axis=1))) * 1000),
            'joint_rms_deg': float(np.rad2deg(np.sqrt(np.mean(
                (simulation.actual[ink] - reference[ink]) ** 2)))),
        }
        if simulation.controller:
            results[mode].update({
                'fallbacks': simulation.controller.fallbacks,
                'active_joint_qps': simulation.controller.active_qps,
                'solve_mean_ms': float(np.mean(simulation.controller.solve_times) * 1000),
                'solve_p99_ms': float(np.percentile(simulation.controller.solve_times, 99) * 1000),
            })
    results['pen_xyz_improvement_percent'] = 100 * (
        1 - results['on']['pen_xyz_rms_mm'] / results['off']['pen_xyz_rms_mm']
    ) if results['off']['pen_xyz_rms_mm'] > 0 else None
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Rebuild shared MATLAB inputs and Python golden runs; no running ROS needed."""
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.io import savemat

ROOT = Path(__file__).resolve().parents[2]
for relative in ("nexarm_description", "algorithm/mpc", "algorithm/pid", "algorithm/lqr"):
    sys.path.insert(0, str(ROOT / relative))

from nexarm_description.draw_s import (  # noqa: E402
    GRIPPER_HOLD, PEN_FORWARD, PEN_LENGTH, chain_from_xml, forward, plan,
)
from nexarm_mpc.simulation import JointTrackingSimulation, arrival_errors  # noqa: E402
import nexarm_mpc.linear_incremental as mpc_module  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "matlab/data")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    # The six chain joints and their limits are literal XML in the source Xacro.
    # No mesh expansion, package index, xacro process or ROS installation needed.
    source = ROOT / "nexarm_description/urdf/nexarm.urdf.xacro"
    xml = source.read_text()
    chain = chain_from_xml(xml)
    root = ET.fromstring(xml)
    limits = [j[3] for j in chain[:5]] + [root.find("joint[@name='gripper_base_joint']/limit")]
    lower, upper, velocity = (np.array([float(j.get(key)) for j in limits])
                              for key in ("lower", "upper", "velocity"))
    q, tips, ink = plan(chain)
    reference = np.column_stack((q, np.zeros(len(q)), np.full(len(q), GRIPPER_HOLD)))
    errors = np.column_stack((arrival_errors(len(q), 5, 1.0, 42), np.zeros(len(q))))
    model = dict(lower=lower, upper=upper, velocity=velocity,
                 origins=np.stack([j[0] for j in chain], axis=2),
                 axes=np.stack([j[1] for j in chain]),
                 revolute=np.array([j[2] == "revolute" for j in chain]),
                 pen_length=PEN_LENGTH, pen_forward=PEN_FORWARD,
                 world_offset=np.fromstring(root.find("joint[@name='world_to_base_joint']/origin").get("xyz"), sep=" "))
    savemat(args.output_dir / "reference.mat", dict(
        reference=reference, reference_tips=tips, ink=ink, time=np.arange(len(q))/30,
        noise_unit=errors, seed=42, dt=1/30, model=model), do_compression=True)
    golden = {}
    original_minimize = mpc_module.minimize

    def tight_minimize(*args, **kwargs):
        # Independent high-accuracy SLSQP reference for the same QP. Preserve
        # the original Python baseline as well; no project source is patched.
        kwargs["options"] = dict(kwargs.get("options", {}), ftol=1e-14, maxiter=500)
        return original_minimize(*args, **kwargs)

    for label in ("off", "pid", "lqr", "mpc", "mpc_tight"):
        mode = "mpc" if label == "mpc_tight" else label
        mpc_module.minimize = tight_minimize if label == "mpc_tight" else original_minimize
        sim = JointTrackingSimulation(reference, lower, upper, velocity, error_deg=.2, seed=42,
                                      control_mode=mode, motor_tau=[.6,.5,.6,.4,0,0], error_joints=5)
        actual_tips = []
        for _ in reference:
            actual = sim.step()
            pose = forward(chain, actual)
            actual_tips.append(pose[:3, 3] + PEN_FORWARD*pose[:3, 0] - PEN_LENGTH*pose[:3, 2])
        golden[label] = dict(actual=sim.actual, commands=sim.commands, tips=np.array(actual_tips),
                            fallbacks=sim.controller.fallbacks if sim.controller else 0)
        diff = np.array(actual_tips)[ink] - tips[ink]
        print(f"{label}: {len(q)} samples, pen RMS={np.sqrt(np.mean(np.sum(diff**2,axis=1)))*1000:.8f} mm", flush=True)
    mpc_module.minimize = original_minimize
    savemat(args.output_dir / "python_baseline.mat", golden, do_compression=True)
    sources = [source, ROOT / "nexarm_description/nexarm_description/draw_s.py",
               ROOT / "algorithm/mpc/nexarm_mpc/linear_incremental.py",
               ROOT / "algorithm/mpc/nexarm_mpc/simulation.py",
               ROOT / "algorithm/pid/nexarm_pid/controller.py", ROOT / "algorithm/lqr/nexarm_lqr/controller.py"]
    metadata = dict(samples=len(q), dt=1/30, seed=42, error_deg=.2,
                    motor_tau=[.6,.5,.6,.4,0,0], text="ee528 / WU Dihang",
                    frame="base_link; add model.world_offset for world coordinates",
                    noise="noise_unit is radians at error_deg=1; multiply by desired degrees",
                    sources={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (args.output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2)+"\n")


if __name__ == "__main__":
    main()

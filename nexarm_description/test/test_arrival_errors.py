"""Motor arrival error must affect joint feedback and the visible pen together."""
from pathlib import Path

import numpy as np
import pytest

from nexarm_mpc.simulation import JointTrackingSimulation

from nexarm_description.draw_s import (
    PEN_FORWARD, PEN_LENGTH, chain_from_xml, forward, plan, simulate_arrival_errors,
)


def test_arrival_error_bounds_and_actual_tip():
    model = Path(__file__).parents[1] / 'urdf' / 'nexarm.urdf.xacro'
    chain = chain_from_xml(model.read_text())
    planned, ideal_tips, ink = plan(chain)
    expected = np.column_stack((planned, np.zeros(len(planned))))
    clean, clean_tips = simulate_arrival_errors(chain, planned, 0.0)
    assert np.array_equal(clean, expected)
    assert np.allclose(clean_tips, ideal_tips)
    actual, tips = simulate_arrival_errors(chain, planned, 1.5, 42)
    repeat, repeated_tips = simulate_arrival_errors(chain, planned, 1.5, 42)
    other, _ = simulate_arrival_errors(chain, planned, 1.5, 43)
    assert np.array_equal(actual, repeat)
    assert np.array_equal(tips, repeated_tips)
    assert not np.array_equal(actual, other)
    # The new online MPC-off plant must exactly preserve the previous noise mode.
    simulation = JointTrackingSimulation(
        expected, [float(j[3].get('lower')) for j in chain[:5]],
        [float(j[3].get('upper')) for j in chain[:5]],
        [float(j[3].get('velocity')) for j in chain[:5]], error_deg=1.5, seed=42, motor_tau=0.0)
    for _ in expected:
        simulation.step()
    assert np.array_equal(actual, simulation.actual)
    error = actual - expected
    assert np.max(np.abs(error)) <= np.deg2rad(1.5) + 1e-12
    assert np.all(np.std(error, axis=0) > np.deg2rad(.1))
    for i, joint in enumerate(chain[:5]):
        assert np.all(actual[:, i] >= float(joint[3].get('lower')))
        assert np.all(actual[:, i] <= float(joint[3].get('upper')))
    for q, tip in zip(actual[::100], tips[::100]):
        pose = forward(chain, q)
        assert np.allclose(tip, pose[:3, 3] + PEN_FORWARD * pose[:3, 0] - PEN_LENGTH * pose[:3, 2])
    assert np.sqrt(np.mean(np.sum((tips[ink] - ideal_tips[ink]) ** 2, axis=1))) > .001
    # Arrival error also changes physical Z; no projection hides that deviation.
    assert np.std(tips[ink, 2]) > .001
    for invalid in [-1., float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            simulate_arrival_errors(chain, planned, invalid)
    with pytest.raises(ValueError):
        simulate_arrival_errors(chain, planned, seed=-1)

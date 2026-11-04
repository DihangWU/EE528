"""Check active constraints, causality, reset, and disturbance rejection."""
import numpy as np
import pytest

from nexarm_mpc.linear_incremental import LinearConstrainedIncrementalMPC
from nexarm_mpc.simulation import JointTrackingSimulation


def test_active_constraints_are_enforced_by_qp():
    controller = LinearConstrainedIncrementalMPC(
        [-1], [1], [.15], dt=.1, horizon=6, control_horizon=4)
    measured = np.array([0.])
    previous = np.array([0.])
    command, info = controller.command(measured, previous, np.full((6, 1), .8))
    assert controller.active_qps == 1 and controller.fallbacks == 0
    assert info['success'].all()
    assert 0 < command[0] <= .015 + 1e-8
    assert np.max(np.abs(info['increments'])) <= .015 + 1e-8
    assert np.all(np.abs(info['commands']) <= 1 + 1e-8)
    assert np.all(np.abs(info['predictions']) <= 1 + 1e-8)
    output_deltas = np.diff(np.vstack((measured, info['predictions'])), axis=0)
    assert np.max(np.abs(output_deltas)) <= .015 + 1e-8


def test_infeasible_output_constraints_use_reported_bounded_fallback():
    controller = LinearConstrainedIncrementalMPC(
        [-1], [1], [.1], dt=.1, horizon=4, control_horizon=3)
    command, info = controller.command([1.], [.9], np.zeros((4, 1)))
    assert controller.fallbacks == 1
    assert not info['success'][0]
    assert .89 - 1e-8 <= command[0] <= .91 + 1e-8


def test_causal_disturbance_rejection_and_reset():
    reference = np.zeros((1500, 2))
    kwargs = dict(reference=reference, lower=[-1, -1], upper=[1, 1],
                  velocity=[3, 3], error_deg=.2, seed=42, motor_tau=0.0)
    baseline = JointTrackingSimulation(**kwargs)
    corrected = JointTrackingSimulation(**kwargs, mpc_enabled=True)
    for _ in reference:
        baseline.step()
        corrected.step()
    assert np.array_equal(baseline._errors, corrected._errors)
    rms_off = np.sqrt(np.mean(baseline.actual ** 2))
    rms_on = np.sqrt(np.mean(corrected.actual ** 2))
    assert rms_on < .98 * rms_off
    assert corrected.controller.fallbacks == 0
    recorded = corrected.actual.copy()
    recorded_commands = corrected.commands.copy()
    corrected.reset()
    assert corrected.index == -1 and corrected.controller.steps == 0
    # Replace only future noise: previous commands must stay identical.
    corrected._errors[100:] *= -1
    for _ in range(100):
        corrected.step()
    assert np.array_equal(recorded_commands[:100], corrected.commands[:100])
    assert np.array_equal(recorded[:100], corrected.actual[:100])


def test_zero_error_baseline_and_invalid_configuration():
    reference = np.column_stack((np.linspace(-.1, .1, 100), np.zeros(100)))
    simulation = JointTrackingSimulation(reference, [-1, -1], [1, 1], [3, 3],
                                         error_deg=0, motor_tau=0.0)
    for _ in reference:
        simulation.step()
    assert np.array_equal(simulation.actual, reference)
    for horizon, control in [(0, 1), (4, 5)]:
        with pytest.raises(ValueError):
            LinearConstrainedIncrementalMPC([-1], [1], [3], horizon=horizon,
                                           control_horizon=control)

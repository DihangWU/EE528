"""Physical first-order response, matched prediction, and fair MPC comparison."""
import numpy as np
import pytest

from nexarm_mpc.linear_incremental import LinearConstrainedIncrementalMPC
from nexarm_mpc.simulation import JointTrackingSimulation


def test_motor_step_response_matches_time_constant_and_zero_tau():
    reference = np.ones((40, 1))
    reference[0] = 0
    simulation = JointTrackingSimulation(reference, [-2], [2], [3], error_deg=0,
                                         motor_tau=.2)
    for _ in reference:
        simulation.step()
    assert 0 < simulation.actual[1, 0] < 1
    assert np.all(np.diff(simulation.actual[:, 0]) >= 0)
    # Six updates at 30 Hz are 0.2 s: one time constant reaches 1-exp(-1).
    assert simulation.actual[6, 0] == pytest.approx(1 - np.exp(-1), abs=1e-12)
    instant = JointTrackingSimulation(reference, [-2], [2], [3], error_deg=0,
                                      motor_tau=0)
    for _ in reference:
        instant.step()
    assert np.array_equal(instant.actual, reference)


def test_lag_prediction_matches_rollout_and_constraints():
    controller = LinearConstrainedIncrementalMPC(
        [-1], [1], [.4], motor_tau=.2, horizon=8, control_horizon=5)
    measured = np.array([0.])
    _, info = controller.command(measured, [0.], np.full((8, 1), .7))
    assert controller.active_qps == 1
    assert controller.fallbacks == 0 and info['success'].all()
    x = measured.copy()
    predicted = []
    for command in info['commands']:
        x = controller.a * x + (1 - controller.a) * command
        predicted.append(x.copy())
    assert np.allclose(predicted, info['predictions'], atol=1e-10)
    assert np.max(np.abs(info['increments'])) <= .4 / 30 + 1e-8
    output_deltas = np.diff(np.vstack((measured, info['predictions'])), axis=0)
    assert np.max(np.abs(output_deltas)) <= .4 / 30 + 1e-8


def test_mpc_compensates_same_lag_and_noise_causally():
    t = np.arange(600) / 30
    reference = (.2 * np.sin(2 * np.pi * .5 * t))[:, None]
    kwargs = dict(reference=reference, lower=[-1], upper=[1], velocity=[3.14],
                  error_deg=.2, seed=42, motor_tau=.2)
    baseline = JointTrackingSimulation(**kwargs)
    corrected = JointTrackingSimulation(**kwargs, mpc_enabled=True)
    for _ in reference:
        baseline.step()
        corrected.step()
    assert np.array_equal(baseline.a, corrected.a)
    assert np.array_equal(baseline._errors, corrected._errors)
    rms_off = np.sqrt(np.mean((baseline.actual - reference) ** 2))
    rms_on = np.sqrt(np.mean((corrected.actual - reference) ** 2))
    assert rms_on < .2 * rms_off
    assert corrected.controller.fallbacks == 0
    recorded_commands = corrected.commands.copy()
    recorded_actual = corrected.actual.copy()
    corrected.reset()
    corrected._errors[100:] *= -1
    for _ in range(100):
        corrected.step()
    assert np.array_equal(recorded_commands[:100], corrected.commands[:100])
    assert np.array_equal(recorded_actual[:100], corrected.actual[:100])
    # Model observer and simulated nominal motor agree without hidden-state access.
    assert np.allclose(corrected.controller.nominal_state, corrected.nominal)


def test_invalid_lag_parameters():
    for tau in [-.1, float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            JointTrackingSimulation(np.zeros((10, 1)), [-1], [1], [3], motor_tau=tau)
        with pytest.raises(ValueError):
            LinearConstrainedIncrementalMPC([-1], [1], [3], motor_tau=tau)

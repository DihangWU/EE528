"""Independent time constants must affect all six modeled servos, including grip."""
import numpy as np
import pytest

from nexarm_mpc.linear_incremental import LinearConstrainedIncrementalMPC, resolve_motor_taus
from nexarm_mpc.simulation import JointTrackingSimulation, arrival_errors


def test_independent_six_servo_step_response():
    reference = np.tile(np.arange(1, 7) / 10, (30, 1))
    reference[0] = 0
    taus = np.array([0., .05, .1, .2, .4, .8])
    simulation = JointTrackingSimulation(reference, [-1] * 6, [1] * 6, [3] * 6,
                                         error_deg=0, motor_tau=taus)
    simulation.step()
    first = simulation.step()
    assert first[0] == reference[1, 0]
    assert np.allclose(first, reference[1] * (1 - simulation.a))
    ratios = first / reference[1]
    assert np.all(np.diff(ratios) < 0)
    assert 0 < first[5] < reference[1, 5]  # Sixth servo is actually dynamic.


def test_uniform_times_equal_scalar_and_keep_five_servo_noise():
    reference = np.zeros((100, 6))
    reference[:, :5] = .05 * np.sin(np.arange(100)[:, None] / 15)
    reference[:, 5] = .4
    kwargs = dict(reference=reference, lower=[-1] * 6, upper=[1] * 6,
                  velocity=[3] * 6, error_joints=5, mpc_enabled=True)
    uniform = JointTrackingSimulation(**kwargs, motor_tau=.2)
    separate = JointTrackingSimulation(**kwargs, motor_tau=[.2] * 6)
    assert np.array_equal(uniform._errors[:, :5], arrival_errors(100, 5))
    assert np.all(uniform._errors[:, 5] == 0)
    for _ in reference:
        uniform.step(); separate.step()
    assert np.array_equal(uniform.actual, separate.actual)
    assert np.allclose(uniform.actual[:, 5], .4)


def test_per_servo_qp_predictions_match_six_motor_rollout():
    controller = LinearConstrainedIncrementalMPC(
        [-1] * 6, [1] * 6, [.4] * 6, motor_tau=[0, .05, .1, .2, .4, .8],
        horizon=8, control_horizon=5)
    _, info = controller.command(np.zeros(6), np.zeros(6), np.full((8, 6), .7))
    assert info['success'].all() and controller.fallbacks == 0
    x = np.zeros(6); predicted = []
    for command in info['commands']:
        x = controller.a * x + (1 - controller.a) * command
        predicted.append(x.copy())
    assert np.allclose(info['predictions'], predicted, atol=1e-9)
    assert np.max(np.abs(info['increments'])) <= .4 / 30 + 1e-8


def test_legacy_uniform_and_individual_override_validation():
    assert np.array_equal(resolve_motor_taus(.2, [-1] * 6), [.2] * 6)
    assert np.array_equal(resolve_motor_taus(.2, [-1, 0, .1, .3, .4, .5]),
                          [.2, 0, .1, .3, .4, .5])
    for invalid in [[0] * 5, [0] * 5 + [float('nan')], [-.5] * 6]:
        with pytest.raises(ValueError):
            resolve_motor_taus(.2, invalid)
    with pytest.raises(ValueError):
        JointTrackingSimulation(np.zeros((10, 6)), [-1] * 6, [1] * 6,
                                [3] * 6, motor_tau=[.2] * 5)

"""Fair controller comparison, limits, feedback causality and replay reset."""
import numpy as np
import pytest
from nexarm_mpc.simulation import JointTrackingSimulation
from nexarm_lqr.controller import LQRController


@pytest.mark.parametrize('mode', ['pid', 'lqr', 'mpc'])
def test_mode_reduces_lag_with_identical_disturbances_and_replays(mode):
    t = np.arange(360) / 30
    reference = np.column_stack([.15 * np.sin(t + j * .1) for j in range(6)])
    options = dict(reference=reference, lower=[-1.] * 6, upper=[1.] * 6,
                   velocity=[3.14] * 6, motor_tau=[.6, .5, .6, .4, 0, 0],
                   error_deg=.2, seed=42, error_joints=5)
    baseline = JointTrackingSimulation(**options)
    run = JointTrackingSimulation(**options, control_mode=mode)
    assert np.array_equal(run._errors, baseline._errors)
    for _ in reference:
        baseline.step()
        run.step()
    assert np.sqrt(np.mean((run.actual - reference) ** 2)) < np.sqrt(
        np.mean((baseline.actual - reference) ** 2))
    assert np.all(run.commands >= -1) and np.all(run.commands <= 1)
    assert np.max(np.abs(np.diff(run.commands, axis=0))) <= 3.14 / 30 + 1e-8
    original_commands, original_actual = run.commands.copy(), run.actual.copy()
    run.reset()
    assert run.controller.steps == 0
    run._errors[100:] *= -1
    for _ in range(100):
        run.step()
    assert np.array_equal(run.commands[:100], original_commands[:100])
    assert np.array_equal(run.actual[:100], original_actual[:100])


@pytest.mark.parametrize('mode', ['pid', 'lqr'])
def test_saturated_step_is_bounded_and_settles(mode):
    reference = np.zeros((900, 2))
    reference[1:450] = [.7, -.7]
    run = JointTrackingSimulation(reference, [-1]*2, [1]*2, [.8]*2,
                                  motor_tau=[.6, 0], error_deg=0, control_mode=mode)
    for _ in reference:
        run.step()
    assert np.all(np.isfinite(run.actual))
    assert np.max(np.abs(run.commands)) <= 1
    assert np.max(np.abs(np.diff(run.commands, axis=0))) <= .8/30 + 1e-10
    assert np.max(np.abs(run.actual[-1])) < .002


def test_lqr_gain_stabilizes_each_declared_motor():
    controller = LQRController([-1]*6, [1]*6, [3]*6, [.6, .5, .6, .4, 0, 0])
    assert np.all(np.abs(controller.a - controller.b * controller.gain) < 1)


def test_invalid_or_conflicting_mode():
    args = (np.zeros((10, 1)), [-1], [1], [3])
    with pytest.raises(ValueError):
        JointTrackingSimulation(*args, control_mode='unknown')
    with pytest.raises(ValueError):
        JointTrackingSimulation(*args, control_mode='pid', mpc_enabled=True)

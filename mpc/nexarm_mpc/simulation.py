"""Causal simulated motor feedback shared by MPC-on and MPC-off runs."""
import math

import numpy as np

from .linear_incremental import LinearConstrainedIncrementalMPC


def arrival_errors(count, joints=5, error_deg=0.2, seed=42):
    """Bounded correlated disturbances, identical to the original error mode."""
    if not math.isfinite(error_deg) or error_deg < 0:
        raise ValueError('error_deg must be finite and nonnegative')
    if not isinstance(seed, int) or seed < 0:
        raise ValueError('error_seed must be a nonnegative integer')
    rng = np.random.default_rng(seed)
    innovations = rng.uniform(-math.radians(error_deg), math.radians(error_deg),
                              (count, joints))
    errors = np.zeros_like(innovations)
    previous = np.zeros(joints)
    for index, innovation in enumerate(innovations):
        previous = .35 * previous + .65 * innovation
        errors[index] = previous
    return errors


class JointTrackingSimulation:
    """Advance one closed-loop sample at a time; cache actual feedback for replay.

    The controller receives only previous measured angle and previous command.
    The plant adds the next random error *after* the command has been chosen.
    This is an ideal angle servo with arrival error, not Gazebo or real hardware.
    """

    def __init__(self, reference, lower, upper, velocity, error_deg=0.2, seed=42,
                 mpc_enabled=False, horizon=12, control_horizon=8):
        self.reference = np.asarray(reference, dtype=float).copy()
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        if (self.reference.ndim != 2 or len(self.reference) == 0
                or self.reference.shape[1] != len(self.lower)
                or self.lower.shape != self.upper.shape
                or not np.all(np.isfinite(self.reference))
                or np.any(self.reference < self.lower) or np.any(self.reference > self.upper)):
            raise ValueError('Reference must be finite and inside joint limits')
        self._errors = arrival_errors(len(reference), len(lower), error_deg, seed)
        self.controller = (LinearConstrainedIncrementalMPC(
            lower, upper, velocity, horizon=horizon, control_horizon=control_horizon)
            if mpc_enabled else None)
        self.reset()

    def reset(self):
        self.index = -1
        self.actual = np.full_like(self.reference, np.nan)
        self.commands = np.full_like(self.reference, np.nan)
        self.previous_command = self.reference[0].copy()
        self.measured = self.reference[0].copy()
        if self.controller:
            self.controller.reset_statistics()

    def step(self):
        """Advance exactly one 1/30-second sample and return measured joints."""
        index = self.index + 1
        if index >= len(self.reference):
            return self.measured.copy()
        if self.controller:
            horizon = self.controller.horizon
            indices = np.minimum(np.arange(index, index + horizon), len(self.reference) - 1)
            command, _ = self.controller.command(
                self.measured, self.previous_command, self.reference[indices])
        else:
            command = self.reference[index].copy()
        self.measured = np.clip(command + self._errors[index], self.lower, self.upper)
        self.previous_command = command.copy()
        self.actual[index] = self.measured
        self.commands[index] = command
        self.index = index
        return self.measured.copy()

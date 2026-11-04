"""Discrete LQR for independent first-order servo states, with output limiting."""
import time
import numpy as np
from scipy.linalg import solve_discrete_are


class LQRController:
    horizon = 1

    def __init__(self, lower, upper, velocity, motor_tau=.2, dt=1 / 30,
                 state_weight=20., command_weight=.2):
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        self.delta = np.asarray(velocity, float) * dt
        tau = np.broadcast_to(np.asarray(motor_tau, float), self.lower.shape)
        if (self.lower.ndim != 1 or self.upper.shape != self.lower.shape
                or self.delta.shape != self.lower.shape
                or not np.all(np.isfinite([self.lower, self.upper, self.delta, tau]))
                or np.any(self.lower >= self.upper) or np.any(self.delta <= 0)
                or np.any(tau < 0) or not np.isfinite(dt) or dt <= 0
                or not np.all(np.isfinite([state_weight, command_weight]))
                or min(state_weight, command_weight) <= 0):
            raise ValueError('Invalid LQR model, costs or joint limits')
        self.a = np.zeros_like(tau)
        positive = tau > 0
        self.a[positive] = np.exp(-dt / tau[positive])
        self.b = 1 - self.a
        self.gain = np.empty_like(tau)
        for j, (a, b) in enumerate(zip(self.a, self.b)):
            p = solve_discrete_are([[a]], [[b]], [[state_weight]], [[command_weight]])[0, 0]
            self.gain[j] = b * p * a / (command_weight + b * p * b)
        self.reset_statistics()

    def reset_statistics(self):
        self.steps = self.fallbacks = self.active_qps = 0
        self.solve_times = []

    def command(self, measured, previous_command, reference):
        started = time.perf_counter()
        target = np.asarray(reference, float)[0]
        # Equilibrium command is the target. Feedback regulates measured-target.
        raw = target - self.gain * (np.asarray(measured, float) - target)
        low = np.maximum(self.lower, np.asarray(previous_command) - self.delta)
        high = np.minimum(self.upper, np.asarray(previous_command) + self.delta)
        command = np.clip(raw, low, high)
        self.steps += 1
        self.solve_times.append(time.perf_counter() - started)
        return command, {'saturated': raw != command}

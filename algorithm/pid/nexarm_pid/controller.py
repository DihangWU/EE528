"""Position feedforward with filtered PID feedback and conditional anti-windup."""
import time
import numpy as np


class PIDController:
    horizon = 1

    def __init__(self, lower, upper, velocity, dt=1 / 30, kp=.6, ki=.5, kd=.01):
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        self.delta = np.asarray(velocity, float) * dt
        if (self.lower.ndim != 1 or self.upper.shape != self.lower.shape
                or self.delta.shape != self.lower.shape
                or not np.all(np.isfinite([self.lower, self.upper, self.delta]))
                or np.any(self.lower >= self.upper) or np.any(self.delta <= 0)
                or not np.isfinite(dt) or dt <= 0
                or not np.all(np.isfinite([kp, ki, kd])) or min(kp, ki, kd) < 0):
            raise ValueError('Invalid PID gains, sample time or joint limits')
        self.dt, self.kp, self.ki, self.kd = dt, kp, ki, kd
        self.reset_statistics()

    def reset_statistics(self):
        self.integral = np.zeros_like(self.lower)
        self.derivative = np.zeros_like(self.lower)
        self.previous_error = None
        self.steps = self.fallbacks = self.active_qps = 0
        self.solve_times = []

    def command(self, measured, previous_command, reference):
        started = time.perf_counter()
        target = np.asarray(reference, float)[0]
        error = target - np.asarray(measured, float)
        if self.previous_error is not None:
            self.derivative = .8 * self.derivative + .2 * (error - self.previous_error) / self.dt
        integral = self.integral + error * self.dt
        raw = target + self.kp * error + self.ki * integral + self.kd * self.derivative
        low = np.maximum(self.lower, np.asarray(previous_command) - self.delta)
        high = np.minimum(self.upper, np.asarray(previous_command) + self.delta)
        command = np.clip(raw, low, high)
        # Do not accumulate integral in the direction that worsens saturation.
        self.integral = np.where((raw - command) * error > 0, self.integral, integral)
        self.previous_error = error.copy()
        self.steps += 1
        self.solve_times.append(time.perf_counter() - started)
        return command, {'saturated': raw != command}

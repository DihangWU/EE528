"""Linear constrained incremental MPC, solved as separable convex QPs."""
import math
import time

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize


def motor_coefficients(motor_tau, joints, dt=1 / 30):
    """Accept a shared scalar or exactly one response time per servo."""
    tau = np.asarray(motor_tau, dtype=float)
    if tau.ndim == 0:
        tau = np.full(joints, float(tau))
    if tau.shape != (joints,) or not np.all(np.isfinite(tau)) or np.any(tau < 0):
        raise ValueError(f'motor_tau must be nonnegative and finite: scalar or {joints} values')
    a = np.zeros(joints)
    positive = tau > 0
    a[positive] = np.exp(-dt / tau[positive])
    return tau, a


def resolve_motor_taus(uniform_tau, overrides):
    """A -1 override inherits the legacy shared motor_tau parameter."""
    uniform, _ = motor_coefficients(uniform_tau, 1)
    values = np.asarray(overrides, dtype=float)
    if (values.shape != (6,) or not np.all(np.isfinite(values))
            or np.any((values < 0) & (values != -1))):
        raise ValueError('Six per-servo times must be >= 0, or -1 to inherit motor_tau')
    return np.where(values == -1, uniform[0], values)


class LinearConstrainedIncrementalMPC:
    """Optimize command increments; apply only the first receding-horizon move.

    Plant: x[k] = a*x[k-1] + (1-a)*u[k], y[k] = x[k] + d[k],
    a = exp(-dt/motor_tau), d[k+1] = rho*d[k] + innovation[k+1].
    A causal nominal-state observer propagates past issued commands; the measured
    residual y[k-1]-x_hat[k-1] is the only disturbance observation. Predictions
    include the lagged motor response to cumulative command increments.
    Cost: sum(Q*(Y-R)**2) + increment_weight*sum(DeltaU**2).
    Constraints: joint command position, predicted output position, command
    increment, and predicted output velocity (including the first measured step).
    """

    def __init__(self, lower, upper, velocity, dt=1 / 30, horizon=12,
                 control_horizon=8, increment_weight=0.002, rho=0.35, motor_tau=0.0):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.velocity = np.asarray(velocity, dtype=float)
        if (self.lower.ndim != 1 or self.upper.shape != self.lower.shape
                or self.velocity.shape != self.lower.shape
                or not np.all(np.isfinite([self.lower, self.upper, self.velocity]))
                or np.any(self.lower >= self.upper) or np.any(self.velocity <= 0)):
            raise ValueError('Finite joint bounds and positive velocities required')
        if (not isinstance(horizon, int) or not isinstance(control_horizon, int)
                or not 1 <= control_horizon <= horizon <= 100
                or not math.isfinite(dt) or dt <= 0
                or not math.isfinite(increment_weight) or increment_weight <= 0
                or not math.isfinite(rho) or not 0 <= rho < 1):
            raise ValueError('Invalid MPC horizon, sample time, weight or correlation')
        self.dt, self.horizon, self.control_horizon = dt, horizon, control_horizon
        self.rho, self.increment_weight = rho, increment_weight
        self.motor_tau, self.a = motor_coefficients(motor_tau, len(self.lower), dt)
        self.L = np.tri(horizon, control_horizon)
        self.D = np.eye(horizon) - np.eye(horizon, k=-1)
        # Response to a persistent command increment at each control step.
        rows = np.arange(horizon)[:, None]
        columns = np.arange(control_horizon)[None, :]
        powers = np.maximum(0, rows - columns + 1)
        self.M = np.where((rows >= columns)[None, :, :],
                          1.0 - self.a[:, None, None] ** powers[None, :, :], 0.0)
        self.G = np.einsum("kh,jhc->jkc", self.D, self.M)
        self.decay = rho ** np.arange(1, horizon + 1)
        self.weights = np.ones(horizon)
        self.weights[-1] = 2.0
        self.H = 2 * (np.einsum("jhc,h,jhd->jcd", self.M, self.weights, self.M)
                      + increment_weight * np.eye(control_horizon)[None, :, :])
        self.H_inverse = np.linalg.inv(self.H)
        self.constraint_matrix = np.stack([np.vstack((self.L, m, g))
                                           for m, g in zip(self.M, self.G)])
        self.reset_statistics()

    def reset_statistics(self):
        self.steps = self.active_qps = self.fallbacks = 0
        self.solve_times = []
        self.nominal_state = None

    def command(self, measured, previous_command, reference):
        """Use previous feedback and a reference preview, never future noise."""
        started = time.perf_counter()
        measured = np.asarray(measured, dtype=float)
        previous_command = np.asarray(previous_command, dtype=float)
        reference = np.asarray(reference, dtype=float)
        expected = (self.horizon, len(self.lower))
        if (reference.shape != expected or measured.shape != self.lower.shape
                or previous_command.shape != self.lower.shape
                or not np.all(np.isfinite(reference)) or not np.all(np.isfinite(measured))
                or not np.all(np.isfinite(previous_command))):
            raise ValueError('Invalid measured state, previous command or preview')
        if self.nominal_state is None:
            # The simulated motor starts at the first reference pose.
            # tau=0 preserves the original instantaneous-servo error observer.
            self.nominal_state = np.where(self.motor_tau > 0, measured, previous_command)
        disturbance = measured - self.nominal_state
        powers = self.a[None, :] ** np.arange(1, self.horizon + 1)[:, None]
        free = (powers * self.nominal_state
                + (1 - powers) * previous_command
                + self.decay[:, None] * disturbance)
        increments = np.empty((self.control_horizon, len(self.lower)))
        delta = self.velocity * self.dt
        free_delta = self.D @ free
        free_delta[0] -= measured
        predictions = np.empty_like(free)
        commands = np.empty_like(free)
        success = np.ones(len(self.lower), dtype=bool)
        for joint in range(len(self.lower)):
            matrix = self.constraint_matrix[joint]
            response = self.M[joint]
            hessian = self.H[joint]
            gradient = 2 * response.T @ (self.weights * (free[:, joint] - reference[:, joint]))
            constraint_lower = np.r_[
                np.full(self.horizon, self.lower[joint] - previous_command[joint]),
                self.lower[joint] - free[:, joint], -delta[joint] - free_delta[:, joint]]
            constraint_upper = np.r_[
                np.full(self.horizon, self.upper[joint] - previous_command[joint]),
                self.upper[joint] - free[:, joint], delta[joint] - free_delta[:, joint]]

            def feasible(z):
                az = matrix @ z
                return (np.all(np.abs(z) <= delta[joint] + 1e-8)
                        and np.all(az >= constraint_lower - 1e-8)
                        and np.all(az <= constraint_upper + 1e-8))

            z = -self.H_inverse[joint] @ gradient
            if not feasible(z):
                self.active_qps += 1
                g = gradient
                result = minimize(
                    lambda v: .5 * v @ hessian @ v + g @ v,
                    np.clip(z, -delta[joint], delta[joint]),
                    jac=lambda v: hessian @ v + g, method='SLSQP',
                    bounds=Bounds(-delta[joint], delta[joint]),
                    constraints=LinearConstraint(matrix,
                                                 constraint_lower, constraint_upper),
                    options={'ftol': 1e-10, 'maxiter': 100})
                if result.success and feasible(result.x):
                    z = result.x
                else:
                    # A measured state can make the output constraints infeasible.
                    # Fall back to a rate/position-limited command, never claim a QP solve.
                    self.fallbacks += 1
                    success[joint] = False
                    move = np.clip(reference[0, joint] - disturbance[joint]
                                   - previous_command[joint], -delta[joint], delta[joint])
                    move = np.clip(move, self.lower[joint] - previous_command[joint],
                                   self.upper[joint] - previous_command[joint])
                    z = np.r_[move, np.zeros(self.control_horizon - 1)]
            increments[:, joint] = z
            commands[:, joint] = previous_command[joint] + self.L @ z
            predictions[:, joint] = free[:, joint] + response @ z
        self.nominal_state = np.clip(
            self.a * self.nominal_state + (1 - self.a) * commands[0],
            self.lower, self.upper)
        self.steps += 1
        self.solve_times.append(time.perf_counter() - started)
        return commands[0].copy(), {
            'increments': increments, 'commands': commands,
            'predictions': predictions, 'success': success,
        }

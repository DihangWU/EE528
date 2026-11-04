"""Linear constrained incremental MPC, solved as separable convex QPs."""
import math
import time

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize


class LinearConstrainedIncrementalMPC:
    """Optimize command increments; apply only the first receding-horizon move.

    Plant: y[k] = u[k] + d[k], d[k+1] = rho*d[k] + innovation[k+1].
    The only disturbance observation is the previous measured y minus previous u.
    The innovation is unknown to this controller. Predicted outputs are
    Y = 1*u_previous + L*DeltaU + [rho,...,rho**Np]*d_previous.
    Cost: sum(Q*(Y-R)**2) + increment_weight*sum(DeltaU**2).
    Constraints: joint command position, predicted output position, command
    increment, and predicted output velocity (including the first measured step).
    """

    def __init__(self, lower, upper, velocity, dt=1 / 30, horizon=12,
                 control_horizon=8, increment_weight=0.002, rho=0.35):
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
        self.L = np.tri(horizon, control_horizon)
        self.D = np.eye(horizon) - np.eye(horizon, k=-1)
        self.G = self.D @ self.L
        self.decay = rho ** np.arange(1, horizon + 1)
        self.weights = np.ones(horizon)
        self.weights[-1] = 2.0
        self.H = 2 * (self.L.T @ (self.weights[:, None] * self.L)
                      + increment_weight * np.eye(control_horizon))
        self.H_inverse = np.linalg.inv(self.H)
        self.constraint_matrix = np.vstack((self.L, self.L, self.G))
        self.reset_statistics()

    def reset_statistics(self):
        self.steps = self.active_qps = self.fallbacks = 0
        self.solve_times = []

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
        disturbance = measured - previous_command
        free = previous_command + self.decay[:, None] * disturbance
        gradient = 2 * self.L.T @ (self.weights[:, None] * (free - reference))
        # Exact unconstrained QP solution is used only if all constraints hold.
        increments = -self.H_inverse @ gradient
        delta = self.velocity * self.dt
        free_delta = self.D @ free
        free_delta[0] -= measured
        predictions = np.empty_like(free)
        commands = np.empty_like(free)
        success = np.ones(len(self.lower), dtype=bool)
        for joint in range(len(self.lower)):
            constraint_lower = np.r_[
                np.full(self.horizon, self.lower[joint] - previous_command[joint]),
                self.lower[joint] - free[:, joint], -delta[joint] - free_delta[:, joint]]
            constraint_upper = np.r_[
                np.full(self.horizon, self.upper[joint] - previous_command[joint]),
                self.upper[joint] - free[:, joint], delta[joint] - free_delta[:, joint]]

            def feasible(z):
                az = self.constraint_matrix @ z
                return (np.all(np.abs(z) <= delta[joint] + 1e-8)
                        and np.all(az >= constraint_lower - 1e-8)
                        and np.all(az <= constraint_upper + 1e-8))

            z = increments[:, joint]
            if not feasible(z):
                self.active_qps += 1
                g = gradient[:, joint]
                result = minimize(
                    lambda v: .5 * v @ self.H @ v + g @ v,
                    np.clip(z, -delta[joint], delta[joint]),
                    jac=lambda v: self.H @ v + g, method='SLSQP',
                    bounds=Bounds(-delta[joint], delta[joint]),
                    constraints=LinearConstraint(self.constraint_matrix,
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
            predictions[:, joint] = free[:, joint] + self.L @ z
        self.steps += 1
        self.solve_times.append(time.perf_counter() - started)
        return commands[0].copy(), {
            'increments': increments, 'commands': commands,
            'predictions': predictions, 'success': success,
        }

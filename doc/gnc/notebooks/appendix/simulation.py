"""
Tank drive robot simulation utilities for appendix notebooks.

This module provides:
1. System discretization utilities
2. Unified simulation framework for control with state estimation
"""
import numpy as np
import control as ctl


def discretize_system(A, B, C, D, dt):
    """
    Discretize continuous-time system using zero-order hold.

    Args:
        A, B, C, D: Continuous-time state-space matrices
        dt: Sampling period [s]

    Returns:
        Ad, Bd, Cd, Dd: Discrete-time state-space matrices
    """
    sys_ct = ctl.ss(A, B, C, D)
    sys_dt = ctl.c2d(sys_ct, dt, method="zoh")

    Ad = np.array(sys_dt.A)
    Bd = np.array(sys_dt.B)
    Cd = np.array(sys_dt.C)
    Dd = np.array(sys_dt.D)

    return Ad, Bd, Cd, Dd


class TankDriveSimulation:
    """
    Tank drive robot simulation runner.

    Handles the simulation loop for true system evolution, measurements,
    state estimation, and control.
    """

    def __init__(
        self, system_matrices, filter_obj, controller_obj, Q, R, dt, N, random_seed=42
    ):
        """
        Initialize simulation.

        Args:
            system_matrices: Dict with 'Ad', 'Bd', 'Cd' for discrete-time system
            filter_obj: Kalman filter object (KalmanFilter or ExtendedKalmanFilter)
            controller_obj: Controller object (LQRController or SwitchingLQRController)
            Q: Process noise covariance
            R: Measurement noise covariance
            dt: Time step [s]
            N: Number of simulation steps
            random_seed: Random seed for reproducibility
        """
        self.Ad = system_matrices["Ad"]
        self.Bd = system_matrices["Bd"]
        self.Cd = system_matrices["Cd"]

        self.filter = filter_obj
        self.controller = controller_obj

        self.Q = Q
        self.R = R
        self.dt = dt
        self.N = N

        # Dimensions
        self.n_states = self.Ad.shape[0]
        self.n_inputs = self.Bd.shape[1]
        self.n_outputs = self.Cd.shape[0]

        # Setup noise generators
        self.Q_chol = np.linalg.cholesky(Q)
        self.R_chol = np.linalg.cholesky(R)
        self.rng = np.random.default_rng(random_seed)

        # Storage arrays
        self.x_true = np.zeros((self.n_states, N))
        self.x_hat = np.zeros((self.n_states, N))
        self.y_meas = np.zeros((self.n_outputs, N))
        self.u_control = np.zeros((N, self.n_inputs))
        self.P_hist = np.zeros((self.n_states, N))

    def set_initial_conditions(self, x0_true=None, x0_hat=None):
        """
        Set initial conditions for true state and estimate.

        Args:
            x0_true: Initial true state (default: zeros)
            x0_hat: Initial state estimate (default: zeros)
        """
        if x0_true is None:
            x0_true = np.zeros(self.n_states)
        if x0_hat is None:
            x0_hat = np.zeros(self.n_states)

        self.x_true[:, 0] = x0_true
        self.x_hat[:, 0] = self.filter.get_state()
        self.P_hist[:, 0] = self.filter.get_uncertainty() ** 2

    def run(self, dynamics_func=None, dynamics_params=None):
        """
        Run the simulation loop.

        Args:
            dynamics_func: Optional nonlinear dynamics function for true system
                          If None, uses linear dynamics (Ad, Bd)
            dynamics_params: Dict of additional parameters for dynamics_func
                            (e.g., {'L': 0.5, 'tau': 0.2})

        Returns:
            results: Dict with simulation results
        """
        for k in range(self.N - 1):
            # --- Generate measurement with noise ---
            v_k = self.R_chol @ self.rng.standard_normal(self.n_outputs)
            self.y_meas[:, k] = self.Cd @ self.x_true[:, k] + v_k

            # --- State estimation: predict & update ---
            if k > 0:
                u_prev = self.u_control[k - 1, :]
            else:
                u_prev = np.zeros(self.n_inputs)

            # Predict step (filter handles linear vs nonlinear)
            if hasattr(self.filter, "predict"):
                # Check if EKF (needs additional params) or linear KF
                if dynamics_params is not None:
                    # EKF signature: predict(u, dt, *params)
                    self.filter.predict(u_prev, self.dt, *dynamics_params.values())
                else:
                    # Linear KF signature: predict(u)
                    self.filter.predict(u_prev)

            # Update step
            self.filter.update(self.y_meas[:, k])

            # Store state estimate and covariance
            self.x_hat[:, k] = self.filter.get_state()
            self.P_hist[:, k] = self.filter.get_uncertainty() ** 2

            # --- Control ---
            self.u_control[k, :] = self.controller.compute_control(self.x_hat[:, k])

            # --- True system propagation with process noise ---
            w_k = self.Q_chol @ self.rng.standard_normal(self.n_states)

            if dynamics_func is not None and dynamics_params is not None:
                # Nonlinear dynamics
                self.x_true[:, k + 1] = (
                    dynamics_func(
                        self.x_true[:, k], self.u_control[k, :], self.dt, **dynamics_params
                    )
                    + w_k
                )
            else:
                # Linear dynamics
                self.x_true[:, k + 1] = (
                    self.Ad @ self.x_true[:, k] + self.Bd @ self.u_control[k, :] + w_k
                )

        # Final step
        v_k = self.R_chol @ self.rng.standard_normal(self.n_outputs)
        self.y_meas[:, -1] = self.Cd @ self.x_true[:, -1] + v_k

        # Final estimate
        if hasattr(self.filter, "predict"):
            if dynamics_params is not None:
                # EKF signature: predict(u, dt, *params)
                self.filter.predict(
                    self.u_control[-2, :], self.dt, *dynamics_params.values()
                )
            else:
                # Linear KF signature: predict(u)
                self.filter.predict(self.u_control[-2, :])

        self.filter.update(self.y_meas[:, -1])
        self.x_hat[:, -1] = self.filter.get_state()
        self.P_hist[:, -1] = self.filter.get_uncertainty() ** 2

        # Return results
        return {
            "x_true": self.x_true,
            "x_hat": self.x_hat,
            "y_meas": self.y_meas,
            "u_control": self.u_control,
            "P_hist": self.P_hist,
            "t": np.arange(self.N) * self.dt,
        }

    def get_results(self):
        """Get simulation results as a dictionary."""
        return {
            "x_true": self.x_true,
            "x_hat": self.x_hat,
            "y_meas": self.y_meas,
            "u_control": self.u_control,
            "P_hist": self.P_hist,
            "t": np.arange(self.N) * self.dt,
        }

    def compute_position_error(self):
        """Compute position error over time."""
        return np.sqrt(
            (self.x_true[0, :] - self.x_hat[0, :]) ** 2
            + (self.x_true[1, :] - self.x_hat[1, :]) ** 2
        )

    def compute_statistics(self):
        """Compute simulation statistics."""
        pos_error = self.compute_position_error()

        stats = {
            "final_position_uncertainty": {
                "x": np.sqrt(self.P_hist[0, -1]),
                "y": np.sqrt(self.P_hist[1, -1]),
            },
            "position_error": {
                "mean": np.mean(pos_error),
                "max": np.max(pos_error),
                "final": pos_error[-1],
            },
        }

        return stats

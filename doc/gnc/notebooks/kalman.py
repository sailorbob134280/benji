"""
Extended Kalman Filter with multi-rate sensor fusion.

This module provides a production-ready EKF implementation for the Benji rover
with support for:
- IMU (gyro) measurements at high rate
- Wheel encoder measurements at high rate
- Accelerometer measurements at high rate
- Camera position measurements at low rate

The EKF naturally handles multi-rate sensors: predict at the control rate,
update with whatever measurements are available.

Example usage:
    ekf = MultiRateEKF(L, tau, dt, Q, R_imu, R_camera, x0, P0)

    for k in range(N):
        ekf.predict(u_k)
        ekf.update_imu_encoder(y_imu, u_k)

        if camera_available:
            ekf.update_camera(y_camera)

        x_hat = ekf.get_state()
"""
import numpy as np

from utils import angle_wrap


class MultiRateEKF:
    """
    Extended Kalman Filter with multi-rate sensor fusion.

    State: [x, y, theta, v_l, v_r]
    IMU+Encoder measurements: [theta, theta_dot, v_l, v_r, a_x, a_y]
    Camera measurements: [x, y]

    The accelerometer provides body-frame accelerations:
        a_x = (v_dot_l + v_dot_r) / 2 = (1/2*tau) * ((u_l - v_l) + (u_r - v_r))
        a_y = v_avg * theta_dot (centripetal)
    """

    def __init__(self, L, tau, dt, Q, R_imu, R_camera, x0, P0):
        """
        Initialize the multi-rate EKF.

        Args:
            L: Wheelbase [m]
            tau: Motor time constant [s]
            dt: Time step [s]
            Q: Process noise covariance (5x5)
            R_imu: IMU+encoder measurement noise covariance (6x6)
            R_camera: Camera measurement noise covariance (2x2)
            x0: Initial state estimate [x, y, theta, v_l, v_r]
            P0: Initial covariance (5x5)
        """
        self.L = L
        self.tau = tau
        self.dt = dt
        self.Q = Q
        self.R_imu = R_imu
        self.R_camera = R_camera

        self.x = x0.copy()
        self.P = P0.copy()

    def predict(self, u):
        """
        Predict step using nonlinear dynamics.

        Args:
            u: Control input [u_l, u_r]
        """
        # Compute Jacobian at current state
        F = self._dynamics_jacobian(self.x, u)

        # Propagate state using nonlinear dynamics
        self.x = self._nonlinear_dynamics(self.x, u)
        self.x[2] = angle_wrap(self.x[2])

        # Propagate covariance
        self.P = F @ self.P @ F.T + self.Q

    def update_imu_encoder(self, y, u):
        """
        Update with IMU + encoder + accelerometer measurements.

        Args:
            y: Measurement vector [theta, theta_dot, v_l, v_r, a_x, a_y]
            u: Control input [u_l, u_r] (needed for accelerometer prediction)
        """
        # Predicted measurement
        y_pred = self._imu_measurement(self.x, u)

        # Measurement Jacobian
        H = self._imu_jacobian(self.x, u)

        # Innovation
        innov = y - y_pred
        innov[0] = angle_wrap(innov[0])  # Wrap theta innovation

        # Kalman gain
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update
        self.x = self.x + K @ innov
        self.x[2] = angle_wrap(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_camera(self, y_cam):
        """
        Update with camera position measurements.

        Args:
            y_cam: Measurement vector [x, y]
        """
        # H_camera: [x, y] from state
        H = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0]])

        # Innovation
        y_pred = H @ self.x
        innov = y_cam - y_pred

        # Kalman gain
        S = H @ self.P @ H.T + self.R_camera
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update
        self.x = self.x + K @ innov
        self.x[2] = angle_wrap(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def _nonlinear_dynamics(self, x, u):
        """Propagate state using nonlinear tank drive dynamics."""
        x_pos, y_pos, theta, v_l, v_r = x
        u_l, u_r = u

        v_avg = (v_l + v_r) / 2.0
        x_dot = v_avg * np.cos(theta)
        y_dot = v_avg * np.sin(theta)
        theta_dot = (v_r - v_l) / self.L
        v_l_dot = (-v_l + u_l) / self.tau
        v_r_dot = (-v_r + u_r) / self.tau

        return np.array(
            [
                x_pos + x_dot * self.dt,
                y_pos + y_dot * self.dt,
                theta + theta_dot * self.dt,
                v_l + v_l_dot * self.dt,
                v_r + v_r_dot * self.dt,
            ]
        )

    def _dynamics_jacobian(self, x, u):
        """Jacobian of nonlinear dynamics w.r.t. state."""
        theta, v_l, v_r = x[2], x[3], x[4]
        v_avg = (v_l + v_r) / 2

        F = np.eye(5)
        F[0, 2] = -v_avg * np.sin(theta) * self.dt
        F[0, 3] = 0.5 * np.cos(theta) * self.dt
        F[0, 4] = 0.5 * np.cos(theta) * self.dt
        F[1, 2] = v_avg * np.cos(theta) * self.dt
        F[1, 3] = 0.5 * np.sin(theta) * self.dt
        F[1, 4] = 0.5 * np.sin(theta) * self.dt
        F[2, 3] = -self.dt / self.L
        F[2, 4] = self.dt / self.L
        F[3, 3] = 1 - self.dt / self.tau
        F[4, 4] = 1 - self.dt / self.tau

        return F

    def _imu_measurement(self, x, u):
        """Expected IMU+encoder+accel measurement."""
        theta, v_l, v_r = x[2], x[3], x[4]
        theta_dot = (v_r - v_l) / self.L

        # Accelerations
        a_x = (1 / (2 * self.tau)) * ((u[0] - v_l) + (u[1] - v_r))
        v_avg = (v_l + v_r) / 2
        a_y = v_avg * theta_dot  # Centripetal

        return np.array([theta, theta_dot, v_l, v_r, a_x, a_y])

    def _imu_jacobian(self, x, u):
        """Jacobian of IMU measurement w.r.t. state."""
        v_l, v_r = x[3], x[4]

        H = np.zeros((6, 5))
        H[0, 2] = 1  # theta
        H[1, 3] = -1 / self.L  # theta_dot from v_l
        H[1, 4] = 1 / self.L  # theta_dot from v_r
        H[2, 3] = 1  # v_l
        H[3, 4] = 1  # v_r
        H[4, 3] = -1 / (2 * self.tau)  # a_x from v_l
        H[4, 4] = -1 / (2 * self.tau)  # a_x from v_r
        # a_y = (v_l + v_r)/2 * (v_r - v_l)/L
        H[5, 3] = (v_r - 2 * v_l) / (2 * self.L)
        H[5, 4] = (2 * v_r - v_l) / (2 * self.L)

        return H

    def get_state(self):
        """Return current state estimate."""
        return self.x.copy()

    def get_covariance(self):
        """Return current covariance matrix."""
        return self.P.copy()

    def get_uncertainty(self):
        """Return standard deviations (sqrt of diagonal of P)."""
        return np.sqrt(np.diag(self.P))

    def reset(self, x0, P0):
        """Reset filter state and covariance."""
        self.x = x0.copy()
        self.P = P0.copy()

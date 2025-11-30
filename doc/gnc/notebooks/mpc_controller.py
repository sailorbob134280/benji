"""
Model Predictive Control (MPC) for tank drive robots.

This module provides a QP-based MPC implementation using OSQP for fast,
embeddable trajectory tracking control.

Key Features:
- OSQP solver for 10-100x speedup over scipy.optimize
- Rate penalty support for systems with motor dynamics
- Warm starting from previous solution
- Pure C implementation available for embedded systems

Example:
    >>> from mpc_controller import QPMPC
    >>> 
    >>> mpc = QPMPC(L, tau, dt, Q, R, R_rate, N_horizon=15, u_max=0.5)
    >>> u = mpc.compute_control(x_current, x_ref_horizon, u_ref_horizon)
"""

import numpy as np

try:
    import osqp
    from scipy import sparse
    OSQP_AVAILABLE = True
except ImportError:
    OSQP_AVAILABLE = False

from utils import angle_wrap, state_error


def compute_jacobians_at_state(x_ref: np.ndarray, L: float, tau: float):
    """
    Compute linearized A, B matrices around a reference state.
    
    Args:
        x_ref: Reference state [x, y, theta, v_l, v_r]
        L: Wheelbase [m]
        tau: Motor time constant [s]
        
    Returns:
        A: Continuous-time state Jacobian (5x5)
        B: Continuous-time input Jacobian (5x2)
    """
    theta = x_ref[2]
    v_l, v_r = x_ref[3], x_ref[4]
    v_avg = (v_l + v_r) / 2
    
    A = np.zeros((5, 5))
    A[0, 2] = -np.sin(theta) * v_avg  # ∂ẋ/∂θ
    A[0, 3] = np.cos(theta) / 2       # ∂ẋ/∂v_l
    A[0, 4] = np.cos(theta) / 2       # ∂ẋ/∂v_r
    A[1, 2] = np.cos(theta) * v_avg   # ∂ẏ/∂θ
    A[1, 3] = np.sin(theta) / 2       # ∂ẏ/∂v_l
    A[1, 4] = np.sin(theta) / 2       # ∂ẏ/∂v_r
    A[2, 3] = -1.0 / L                # ∂θ̇/∂v_l
    A[2, 4] = 1.0 / L                 # ∂θ̇/∂v_r
    A[3, 3] = -1.0 / tau              # ∂v̇_l/∂v_l
    A[4, 4] = -1.0 / tau              # ∂v̇_r/∂v_r
    
    B = np.zeros((5, 2))
    B[3, 0] = 1.0 / tau
    B[4, 1] = 1.0 / tau
    
    return A, B


class QPMPC:
    """
    QP-based MPC using OSQP solver.
    
    Key advantages:
    - 10-100x faster than scipy.optimize
    - Warm starting from previous solution
    - Guaranteed convergence (convex problem)
    - Embeddable C code generation (osqp.org)
    
    Rate penalty (R_rate) is critical for systems with motor dynamics -
    penalizing rapid control changes improves tracking.
    """
    
    def __init__(self, L: float, tau: float, dt: float, 
                 Q: np.ndarray, R: np.ndarray, R_rate: np.ndarray,
                 N_horizon: int, u_max: float, exp_weight: float = 1.0):
        """
        Args:
            L: Wheelbase [m]
            tau: Motor time constant [s]
            dt: Time step [s]
            Q: State error cost (5x5)
            R: Control effort cost (2x2)
            R_rate: Control rate cost (2x2) - higher = smoother control
            N_horizon: Prediction horizon (steps)
            u_max: Maximum control input
            exp_weight: Exponential cost weight (1.0 = uniform)
        """
        if not OSQP_AVAILABLE:
            raise ImportError("OSQP not installed. Install with: pip install osqp")
        
        self.L, self.tau, self.dt = L, tau, dt
        self.Q, self.R, self.R_rate = Q, R, R_rate
        self.N = N_horizon
        self.u_max = u_max
        self.exp_weight = exp_weight
        self.n_x, self.n_u = 5, 2
        
        self.u_prev = None
        self.prev_control = np.zeros(2)
        self.solver = None
        
    def _build_prediction_matrices(self, x_current: np.ndarray):
        """Build S, R such that x_{1:N} = S @ x_0 + R @ u_{0:N-1}."""
        Ac, Bc = compute_jacobians_at_state(x_current, self.L, self.tau)
        Ad = np.eye(5) + self.dt * Ac
        Bd = self.dt * Bc
        
        N, nx, nu = self.N, self.n_x, self.n_u
        
        S = np.zeros((N * nx, nx))
        Ad_power = np.eye(nx)
        for i in range(N):
            Ad_power = Ad_power @ Ad
            S[i*nx:(i+1)*nx, :] = Ad_power
        
        R_mat = np.zeros((N * nx, N * nu))
        for i in range(N):
            for j in range(i + 1):
                Ad_pow = np.linalg.matrix_power(Ad, i - j)
                R_mat[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Ad_pow @ Bd
        
        return S, R_mat
    
    def _build_qp(self, x_current, x_ref_horizon, u_ref_horizon):
        """Build QP matrices for OSQP."""
        N, nx, nu = self.N, self.n_x, self.n_u
        
        S, R_pred = self._build_prediction_matrices(x_current)
        x_free = S @ x_current
        
        x_ref_flat = np.zeros(N * nx)
        u_ref_flat = np.zeros(N * nu)
        for k in range(N):
            x_ref_flat[k*nx:(k+1)*nx] = x_ref_horizon[:, min(k+1, x_ref_horizon.shape[1]-1)]
            u_ref_flat[k*nu:(k+1)*nu] = u_ref_horizon[:, min(k, u_ref_horizon.shape[1]-1)]
        
        Q_blocks = [self.exp_weight**k * self.Q for k in range(N)]
        Q_bar = sparse.block_diag(Q_blocks)
        R_bar = sparse.block_diag([self.R] * N)
        
        D = np.zeros((N * nu, N * nu))
        for k in range(N):
            D[k*nu:(k+1)*nu, k*nu:(k+1)*nu] = np.eye(nu)
            if k > 0:
                D[k*nu:(k+1)*nu, (k-1)*nu:k*nu] = -np.eye(nu)
        D = sparse.csc_matrix(D)
        R_rate_bar = sparse.block_diag([self.R_rate] * N)
        
        f = x_free - x_ref_flat
        for k in range(N):
            f[k*nx + 2] = angle_wrap(f[k*nx + 2])
        
        P_state = R_pred.T @ Q_bar @ R_pred
        q_state = R_pred.T @ Q_bar @ f
        P_ctrl = R_bar
        q_ctrl = -R_bar @ u_ref_flat
        
        d_prev = np.zeros(N * nu)
        d_prev[:nu] = self.prev_control
        P_rate = D.T @ R_rate_bar @ D
        q_rate = -D.T @ R_rate_bar @ d_prev
        
        P = sparse.csc_matrix(P_state + P_ctrl.toarray() + P_rate.toarray())
        q = q_state + q_ctrl + q_rate
        
        A_ineq = sparse.eye(N * nu)
        l = -self.u_max * np.ones(N * nu)
        u_upper = self.u_max * np.ones(N * nu)
        
        return P, q, A_ineq, l, u_upper
    
    def compute_control(self, x_current: np.ndarray, 
                        x_ref_horizon: np.ndarray, 
                        u_ref_horizon: np.ndarray) -> np.ndarray:
        """
        Compute optimal control using OSQP.
        
        Args:
            x_current: Current state [x, y, theta, v_l, v_r]
            x_ref_horizon: Reference trajectory (5 x N_horizon+1)
            u_ref_horizon: Reference feedforward (2 x N_horizon)
        
        Returns:
            Optimal control [u_l, u_r]
        """
        P, q, A, l, u_upper = self._build_qp(x_current, x_ref_horizon, u_ref_horizon)
        
        if self.solver is None:
            self.solver = osqp.OSQP()
            self.solver.setup(P, q, A, l, u_upper, 
                            warm_start=True, verbose=False,
                            eps_abs=1e-4, eps_rel=1e-4, max_iter=100)
        else:
            self.solver.update(Px=sparse.triu(P).data, q=q, l=l, u=u_upper)
        
        if self.u_prev is not None:
            u_warm = np.zeros_like(self.u_prev)
            u_warm[:-2] = self.u_prev[2:]
            u_warm[-2:] = self.u_prev[-2:]
            self.solver.warm_start(x=u_warm)
        
        result = self.solver.solve()
        
        if result.info.status != 'solved':
            return u_ref_horizon[:, 0] if u_ref_horizon.shape[1] > 0 else np.zeros(2)
        
        u_opt = result.x[:2]
        self.u_prev = result.x
        self.prev_control = u_opt
        
        return np.clip(u_opt, -self.u_max, self.u_max)
    
    def reset(self):
        """Reset warm-start state."""
        self.u_prev = None
        self.prev_control = np.zeros(2)
        self.solver = None


class NonlinearMPC:
    """
    Nonlinear MPC using scipy.optimize (SLSQP).

    Uses the full nonlinear dynamics for prediction, which is more accurate
    than linearized MPC for trajectories with large heading changes.

    Trade-off vs QPMPC:
    - Slower (~20ms vs ~2ms per solve)
    - More accurate for highly nonlinear trajectories
    - Not suitable for embedded (complex dependencies)
    - Can find local minima (non-convex problem)

    NOTE: This cannot be formulated as a QP because nonlinear dynamics
    make the problem non-convex!
    """

    def __init__(self, dynamics_func, dt: float,
                 Q: np.ndarray, R: np.ndarray,
                 N_horizon: int, N_control: int, u_max: float):
        """
        Args:
            dynamics_func: Nonlinear dynamics function f(x, u, dt) -> x_next
            dt: Time step [s]
            Q: State error cost (5x5)
            R: Control effort cost (2x2)
            N_horizon: Prediction horizon (steps)
            N_control: Control horizon (steps) - reduces variables
            u_max: Maximum control input
        """
        from scipy.optimize import minimize
        self._minimize = minimize

        self.dynamics_func = dynamics_func
        self.dt = dt
        self.Q, self.R = Q, R
        self.N = N_horizon
        self.N_c = N_control
        self.u_max = u_max
        self.n_x, self.n_u = 5, 2

        self.u_prev = None

    def compute_control(self, x_current: np.ndarray,
                        x_ref_horizon: np.ndarray,
                        u_ref_horizon: np.ndarray) -> np.ndarray:
        """
        Compute optimal control using nonlinear optimization.
        
        Args:
            x_current: Current state [x, y, theta, v_l, v_r]
            x_ref_horizon: Reference trajectory (5 x N_horizon+1)
            u_ref_horizon: Reference feedforward (2 x N_horizon)
        
        Returns:
            Optimal control [u_l, u_r]
        """
        def cost(delta_u_flat):
            delta_u = delta_u_flat.reshape(self.N_c, 2)
            
            x_pred = x_current.copy()
            total_cost = 0.0
            
            for k in range(self.N):
                # Control move selection (hold after N_c)
                u_idx = min(k, self.N_c - 1)
                u_k = u_ref_horizon[:, min(k, u_ref_horizon.shape[1]-1)] + delta_u[u_idx]
                
                # Propagate using nonlinear dynamics
                x_pred = self.dynamics_func(x_pred, u_k, self.dt)
                
                # State error with angle wrapping
                x_ref_k = x_ref_horizon[:, min(k+1, x_ref_horizon.shape[1]-1)]
                e = state_error(x_pred, x_ref_k)
                
                # Cost
                total_cost += e @ self.Q @ e + delta_u[u_idx] @ self.R @ delta_u[u_idx]
            
            return total_cost
        
        # Bounds on delta_u
        bounds = [(-self.u_max, self.u_max)] * (self.N_c * 2)
        
        # Initial guess (warm start from previous)
        if self.u_prev is not None:
            delta_u0 = np.zeros(self.N_c * 2)
            delta_u0[:-2] = self.u_prev[2:]
            delta_u0[-2:] = self.u_prev[-2:]
        else:
            delta_u0 = np.zeros(self.N_c * 2)
        
        result = self._minimize(cost, delta_u0, method='SLSQP', bounds=bounds,
                               options={'maxiter': 50, 'ftol': 1e-4})
        
        delta_u_opt = result.x.reshape(self.N_c, 2)
        self.u_prev = result.x
        
        u_total = u_ref_horizon[:, 0] + delta_u_opt[0]
        return np.clip(u_total, -self.u_max, self.u_max)
    
    def reset(self):
        """Reset warm-start state."""
        self.u_prev = None

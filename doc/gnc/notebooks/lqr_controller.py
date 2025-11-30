import numpy as np
import control as ctl


class LQRController:
    """
    Linear Quadratic Regulator (LQR) controller.
    
    Computes optimal state-feedback control: u = -K(x - x_ref)
    with input saturation.
    """
    
    def __init__(self, Ad, Bd, Q, R, u_max=None):
        """
        Initialize LQR controller.
        
        Args:
            Ad: Discrete-time state transition matrix
            Bd: Discrete-time input matrix
            Q: State cost matrix (penalizes state error)
            R: Control cost matrix (penalizes control effort)
            u_max: Control saturation limit (applied symmetrically)
        """
        self.Ad = Ad
        self.Bd = Bd
        self.Q = Q
        self.R = R
        self.u_max = u_max
        
        # Compute LQR gain
        self.K, self.S, self.E = ctl.dlqr(Ad, Bd, Q, R)
        
        # Current reference (can be updated)
        self.x_ref = np.zeros(Ad.shape[0])
        
    def compute_control(self, x_hat, x_ref=None):
        """
        Compute LQR control input.
        
        Args:
            x_hat: Current state estimate
            x_ref: Reference state (optional, uses stored reference if None)
        
        Returns:
            u: Control input (saturated if u_max is set)
        """
        if x_ref is not None:
            self.x_ref = x_ref
            
        # LQR control law: u = -K(x - x_ref)
        x_error = x_hat - self.x_ref
        u = -self.K @ x_error
        
        # Apply saturation if specified
        if self.u_max is not None:
            u = np.clip(u, -self.u_max, self.u_max)
            
        return u
    
    def set_reference(self, x_ref):
        """Set the reference state."""
        self.x_ref = x_ref.copy()
        
    def get_reference(self):
        """Get current reference state."""
        return self.x_ref.copy()
    
    def get_gain(self):
        """Get LQR gain matrix."""
        return self.K.copy()
    
    def get_closed_loop_eigenvalues(self):
        """Get closed-loop eigenvalues (Ad - Bd*K)."""
        return self.E.copy()


class SwitchingLQRController(LQRController):
    """
    LQR controller with reference switching based on distance to target.
    
    Switches between a "moving" reference and a "stop" reference
    when close to the target position.
    """
    
    def __init__(self, Ad, Bd, Q, R, x_ref_pos, x_ref_stop, 
                 target_pos, target_threshold=0.5, u_max=None):
        """
        Initialize switching LQR controller.
        
        Args:
            Ad: Discrete-time state transition matrix
            Bd: Discrete-time input matrix
            Q: State cost matrix
            R: Control cost matrix
            x_ref_pos: Reference state for moving toward target
            x_ref_stop: Reference state for stopping at target
            target_pos: Target position [x_target, y_target]
            target_threshold: Distance threshold for switching to stop reference
            u_max: Control saturation limit
        """
        super().__init__(Ad, Bd, Q, R, u_max)
        
        self.x_ref_pos = x_ref_pos.copy()
        self.x_ref_stop = x_ref_stop.copy()
        self.target_pos = np.array(target_pos)
        self.target_threshold = target_threshold
        
        # Track which reference is active
        self.is_at_target = False
        
    def compute_control(self, x_hat):
        """
        Compute LQR control with automatic reference switching.
        
        Args:
            x_hat: Current state estimate
        
        Returns:
            u: Control input (saturated if u_max is set)
        """
        # Compute distance to target (assumes x_hat[0:2] = [x, y])
        dist_to_target = np.linalg.norm(x_hat[0:2] - self.target_pos)
        
        # Select reference based on distance
        if dist_to_target < self.target_threshold:
            self.x_ref = self.x_ref_stop
            self.is_at_target = True
        else:
            self.x_ref = self.x_ref_pos
            self.is_at_target = False
            
        # Compute control using parent class method
        return super().compute_control(x_hat, self.x_ref)
    
    def set_target(self, target_pos, target_threshold=None):
        """
        Update target position and optionally threshold.
        
        Args:
            target_pos: New target position [x, y]
            target_threshold: New threshold (optional)
        """
        self.target_pos = np.array(target_pos)
        if target_threshold is not None:
            self.target_threshold = target_threshold
            
    def set_references(self, x_ref_pos=None, x_ref_stop=None):
        """
        Update reference states.
        
        Args:
            x_ref_pos: New moving reference (optional)
            x_ref_stop: New stop reference (optional)
        """
        if x_ref_pos is not None:
            self.x_ref_pos = x_ref_pos.copy()
        if x_ref_stop is not None:
            self.x_ref_stop = x_ref_stop.copy()
            
    def get_distance_to_target(self, x_hat):
        """Get current distance to target."""
        return np.linalg.norm(x_hat[0:2] - self.target_pos)
    
    def at_target(self):
        """Check if currently using stop reference."""
        return self.is_at_target

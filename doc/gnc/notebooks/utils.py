"""
Shared utility functions for GNC algorithms.

This module provides common operations used across dynamics, estimation,
and control implementations.
"""
import numpy as np


def angle_wrap(theta):
    """
    Wrap angle to [-pi, pi].

    Args:
        theta: Angle in radians (scalar or array)

    Returns:
        Wrapped angle in [-pi, pi]
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi


def state_error(x: np.ndarray, x_ref: np.ndarray) -> np.ndarray:
    """
    Compute state error with proper heading angle wrapping.

    For tank drive robots, the heading error must be wrapped to [-pi, pi]
    to avoid discontinuities at +/- pi.

    Args:
        x: Current state [x, y, theta, v_l, v_r]
        x_ref: Reference state [x, y, theta, v_l, v_r]

    Returns:
        State error with wrapped heading component
    """
    error = x - x_ref
    error[2] = angle_wrap(error[2])
    return error


def generate_smooth_reference_trajectory(waypoints, t, v_cruise, L, tau, turn_time=1.0):
    """
    Generate reference trajectory with smooth heading transitions.

    Key insight: Include turn phases where the robot adjusts heading!
    Instantaneous heading changes require infinite angular velocity,
    which is physically impossible.

    During turns:
    - Heading changes smoothly (linear ramp)
    - Feedforward includes differential velocity for turning
    - Robot slows down or stops for heading change

    During straight segments:
    - Constant forward velocity
    - Equal left/right wheel speeds

    Args:
        waypoints: Array of (x, y) waypoints, shape (N_waypoints, 2)
        t: Time array
        v_cruise: Nominal cruise speed [m/s]
        L: Wheelbase [m]
        tau: Motor time constant [s]
        turn_time: Time allocated for heading transitions [s]

    Returns:
        x_ref: Reference state trajectory (5, len(t))
               [x, y, theta, v_l, v_r]
        u_ref: Feedforward control trajectory (2, len(t))
               [u_l, u_r]
    """
    N = len(t)
    dt = t[1] - t[0] if len(t) > 1 else 0.05

    x_ref = np.zeros((5, N))
    u_ref = np.zeros((2, N))

    # Compute segments with turn phases
    segments = []
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        dist = np.sqrt(dx**2 + dy**2)
        heading = np.arctan2(dy, dx)
        straight_time = dist / v_cruise if dist > 0.01 else 0.5
        segments.append(
            {
                "start": p1,
                "end": p2,
                "heading": heading,
                "dist": dist,
                "straight_time": straight_time,
            }
        )

    # Build timeline: [turn1, straight1, turn2, straight2, ...]
    # First segment has no initial turn (starts aligned)
    phases = []
    for i, seg in enumerate(segments):
        if i > 0:
            # Turn phase before this segment
            prev_heading = segments[i - 1]["heading"]
            delta_theta = angle_wrap(seg["heading"] - prev_heading)
            phases.append(
                {
                    "type": "turn",
                    "duration": turn_time,
                    "start_heading": prev_heading,
                    "end_heading": seg["heading"],
                    "delta_theta": delta_theta,
                    "position": seg["start"],  # Turn in place
                }
            )
        # Straight phase
        phases.append(
            {
                "type": "straight",
                "duration": seg["straight_time"],
                "heading": seg["heading"],
                "start": seg["start"],
                "end": seg["end"],
            }
        )

    # Generate trajectory points
    phase_start_times = [0]
    for phase in phases[:-1]:
        phase_start_times.append(phase_start_times[-1] + phase["duration"])

    for k in range(N):
        tk = t[k]

        # Find current phase
        phase_idx = 0
        for i, start_t in enumerate(phase_start_times):
            if i + 1 < len(phase_start_times):
                if tk >= start_t and tk < phase_start_times[i + 1]:
                    phase_idx = i
                    break
            else:
                phase_idx = i

        phase = phases[phase_idx]
        t_in_phase = tk - phase_start_times[phase_idx]
        frac = min(t_in_phase / phase["duration"], 1.0) if phase["duration"] > 0 else 1.0

        if phase["type"] == "turn":
            # Turning in place
            x_ref[0, k] = phase["position"][0]
            x_ref[1, k] = phase["position"][1]

            # Smooth heading transition (linear interpolation)
            x_ref[2, k] = phase["start_heading"] + frac * phase["delta_theta"]

            # Angular velocity needed
            omega_needed = phase["delta_theta"] / phase["duration"]

            # omega = (v_r - v_l) / L, keeping forward speed = 0 (turn in place)
            # v_r - v_l = omega * L
            # v_r + v_l = 0
            # => v_r = omega*L/2, v_l = -omega*L/2
            v_turn = omega_needed * L / 2
            x_ref[3, k] = -v_turn  # v_l
            x_ref[4, k] = v_turn  # v_r

            # Feedforward for these velocities
            u_ref[0, k] = -v_turn  # u_l
            u_ref[1, k] = v_turn  # u_r

        else:  # straight
            # Position along segment (linear interpolation)
            x_ref[0, k] = phase["start"][0] + frac * (phase["end"][0] - phase["start"][0])
            x_ref[1, k] = phase["start"][1] + frac * (phase["end"][1] - phase["start"][1])
            x_ref[2, k] = phase["heading"]

            # Velocities for straight-line motion
            x_ref[3, k] = v_cruise  # v_l
            x_ref[4, k] = v_cruise  # v_r

            # Feedforward control
            u_ref[0, k] = v_cruise
            u_ref[1, k] = v_cruise

    return x_ref, u_ref

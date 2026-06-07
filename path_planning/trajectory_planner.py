r"""
trajectory_planner.py
---------------------
eta^3 spline trajectory planner — adds a jerk-limited velocity profile on top
of an eta^3 geometric path.

Given a geometric path (sequence of eta^3 segments), the planner computes a
time-parameterised trajectory that satisfies:
  - Start and end at rest (v=0, a=0)
  - Respects user-specified maximum velocity, acceleration, and jerk
  - Produces a smooth S-curve velocity profile:

      /~~~~~--------------~~~~~\
     /                          \
    /                            \
    pos.|pos.|neg.|  cruise  |neg.|neg.|neg.
    jerk|acc.|jerk|  at vmax |jerk|acc.|jerk
     0    1    2      3(opt)   4    5    6

Type
----
Hybrid — the eta3_trajectory class and velocity profiling logic are adapted
from PythonRobotics (Sakai & Dinius).  The Capstone-specific scenario (test3)
uses original start/end coordinates and heading angles derived from the
building survey geometry.

Usage
-----
    python trajectory_planner.py

References
----------
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
- Dinius blog post: https://jwdinius.github.io/blog/2018/eta3traj
- eta^3 paper: https://ieeexplore.ieee.org/document/4339545/
"""

import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eta3_spline_path import eta3_path, eta3_path_segment  # noqa: E402


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class MaxVelocityNotReached(Exception):
    """Raised when the velocity profile cannot reach the requested maximum."""
    def __init__(self, actual, desired):
        super().__init__(
            f"Actual peak velocity {actual:.4f} does not match desired {desired:.4f}"
        )


# ---------------------------------------------------------------------------
# Trajectory class
# ---------------------------------------------------------------------------

class eta3_trajectory(eta3_path):
    r"""
    Time-parameterised eta^3 trajectory.

    Wraps an eta3_path with a jerk-limited S-curve velocity profile so that
    the robot starts and stops smoothly from rest.

    Parameters
    ----------
    segments : list of eta3_path_segment
    max_vel  : float   Maximum cruising speed (m/s).
    v0       : float   Initial speed (m/s), default 0.
    a0       : float   Initial acceleration (m/s^2), default 0.
    max_accel: float   Maximum acceleration magnitude (m/s^2), default 2.
    max_jerk : float   Maximum jerk magnitude (m/s^3), default 5.
    """

    def __init__(self, segments, max_vel, v0=0.0, a0=0.0,
                 max_accel=2.0, max_jerk=5.0):
        assert max_vel > 0 and v0 >= 0 and a0 >= 0
        assert max_accel > 0 and max_jerk > 0
        assert a0 <= max_accel and v0 <= max_vel
        super().__init__(segments=segments)
        self.total_length = sum(s.segment_length for s in self.segments)
        self.max_vel   = float(max_vel)
        self.v0        = float(v0)
        self.a0        = float(a0)
        self.max_accel = float(max_accel)
        self.max_jerk  = float(max_jerk)
        lengths = np.array([s.segment_length for s in self.segments])
        self.cum_lengths = np.concatenate(([0], np.cumsum(lengths)))
        self._build_velocity_profile()
        self.ui_prev = 0
        self.prev_seg_id = 0

    def _build_velocity_profile(self):
        """Compute the seven-section S-curve timing and length arrays."""
        j  = self.max_jerk
        am = self.max_accel

        delta_a = am - self.a0
        t_s1    = delta_a / j
        v_s1    = self.v0 + self.a0*t_s1 + j*t_s1**2 / 2
        s_s1    = self.v0*t_s1 + self.a0*t_s1**2/2 + j*t_s1**3/6

        t_sf = am / j
        v_sf = j * t_sf**2 / 2
        s_sf = j * t_sf**3 / 6

        # Solve for achievable maximum velocity (may be < requested max_vel)
        a_coeff = 1 / am
        b_coeff = (1.5*am/j + v_s1/am
                   - (am**2/j + v_s1)/am)
        c_coeff = (s_s1 + s_sf - self.total_length
                   - 7*am**3/(3*j**2)
                   - v_s1*(am/j + v_s1/am)
                   + (am**2/j + v_s1/am)**2 / (2*am))
        v_max = (-b_coeff + np.sqrt(b_coeff**2 - 4*a_coeff*c_coeff)) / (2*a_coeff)
        if self.max_vel > v_max:
            self.max_vel = v_max

        T = np.zeros(7)
        V = np.zeros(7)
        S = np.zeros(7)

        # Section 0
        T[0] = t_s1;  V[0] = v_s1;  S[0] = s_s1

        # Section 1
        dv = (self.max_vel - j*(am/j)**2/2) - V[0]
        T[1] = dv / am
        V[1] = V[0] + am*T[1]
        S[1] = V[0]*T[1] + am*T[1]**2/2

        # Section 2
        T[2] = am / j
        V[2] = V[1] + am*T[2] - j*T[2]**2/2
        if not np.isclose(V[2], self.max_vel):
            raise MaxVelocityNotReached(V[2], self.max_vel)
        S[2] = V[1]*T[2] + am*T[2]**2/2 - j*T[2]**3/6

        # Section 4
        T[4] = am / j
        V[4] = self.max_vel - j*T[4]**2/2
        S[4] = self.max_vel*T[4] - j*T[4]**3/6

        # Section 5
        dv = V[4] - v_sf
        T[5] = dv / am
        V[5] = V[4] - am*T[5]
        S[5] = V[4]*T[5] - am*T[5]**2/2

        # Section 6
        T[6] = t_sf
        V[6] = V[5] - j*t_sf**2/2
        if not np.isclose(V[6], 0):
            raise AssertionError(f"Final velocity {V[6]:.6f} is not zero")
        S[6] = s_sf

        # Section 3 (cruise — absorbs remaining length)
        remaining = self.total_length - S.sum()
        if remaining > 0:
            S[3] = remaining
            V[3] = self.max_vel
            T[3] = S[3] / self.max_vel

        assert np.all(T >= 0), "Kinematic limits are too tight for this path length"
        self.times       = T
        self.vels        = V
        self.seg_lengths = S
        self.total_time  = T.sum()

    def _get_interp_param(self, seg_id, s, ui, tol=0.001):
        """Newton-Raphson root-find for the arc-length parameter on a segment."""
        f      = lambda u: self.segments[seg_id].f_length(u)[0] - s
        fprime = lambda u: self.segments[seg_id].s_dot(u)
        while 0 <= ui <= 1 and abs(f(ui)) > tol:
            ui -= f(ui) / fprime(ui)
        return max(0.0, min(ui, 1.0))

    def calc_traj_point(self, t):
        """
        Evaluate the full robot state at time t.

        Returns
        -------
        np.ndarray, shape (5,)
            [x, y, heading, linear_velocity, angular_velocity]
        """
        T = self.times
        V = self.vels
        S = self.seg_lengths
        am = self.max_accel
        j  = self.max_jerk

        if t <= T[0]:
            v = self.v0 + j*t**2/2
            s = self.v0*t + j*t**3/6
            a = j*t
        elif t <= T[:2].sum():
            dt = t - T[0]
            v  = V[0] + am*dt
            s  = S[0] + V[0]*dt + am*dt**2/2
            a  = am
        elif t <= T[:3].sum():
            dt = t - T[:2].sum()
            v  = V[1] + am*dt - j*dt**2/2
            s  = S[:2].sum() + V[1]*dt + am*dt**2/2 - j*dt**3/6
            a  = am - j*dt
        elif t <= T[:4].sum():
            dt = t - T[:3].sum()
            v  = V[3]
            s  = S[:3].sum() + V[3]*dt
            a  = 0.0
        elif t <= T[:5].sum():
            dt = t - T[:4].sum()
            v  = V[3] - j*dt**2/2
            s  = S[:4].sum() + V[3]*dt - j*dt**3/6
            a  = -j*dt
        elif t <= T[:-1].sum():
            dt = t - T[:5].sum()
            v  = V[4] - am*dt
            s  = S[:5].sum() + V[4]*dt - am*dt**2/2
            a  = -am
        elif t < T.sum():
            dt = t - T[:-1].sum()
            v  = V[5] - am*dt + j*dt**2/2
            s  = S[:-1].sum() + V[5]*dt - am*dt**2/2 + j*dt**3/6
            a  = -am + j*dt
        else:
            v, s, a = 0.0, self.total_length, 0.0

        seg_id = int(np.max(np.argwhere(self.cum_lengths <= s)))
        if seg_id == len(self.segments):
            seg_id, ui = len(self.segments) - 1, 1.0
        else:
            ui = self._get_interp_param(seg_id, s - self.cum_lengths[seg_id],
                                        self.ui_prev)

        if seg_id != self.prev_seg_id:
            self.ui_prev = 0
        else:
            self.ui_prev = ui
        self.prev_seg_id = seg_id

        d  = self.segments[seg_id].calc_deriv(ui, order=1)
        dd = self.segments[seg_id].calc_deriv(ui, order=2)
        su = self.segments[seg_id].s_dot(ui)

        if not np.isclose(su, 0) and not np.isclose(v, 0):
            ut  = v / su
            utt = a / su - (d[0]*dd[0] + d[1]*dd[1]) / su**2 * ut
            xt  = d[0]*ut;  yt  = d[1]*ut
            xtt = dd[0]*ut**2 + d[0]*utt
            ytt = dd[1]*ut**2 + d[1]*utt
            omega = (ytt*xt - xtt*yt) / v**2
        else:
            omega = 0.0

        pos = self.segments[seg_id].calc_point(ui)
        return np.array([pos[0], pos[1], math.atan2(d[1], d[0]), v, omega])


# ---------------------------------------------------------------------------
# Capstone scenario
# ---------------------------------------------------------------------------

def run_capstone_scenario(max_vel=1.0, max_accel=1.0, max_jerk=1.0, n_points=1001):
    """
    Compute and plot the trajectory for the Capstone building survey scenario.

    The robot starts at the south face of the building (heading north) and
    sweeps to the east corner (heading west), using the distance between
    start and end poses as the eta shaping parameter.

    Parameters
    ----------
    max_vel   : float  Maximum velocity (m/s).
    max_accel : float  Maximum acceleration (m/s^2).
    max_jerk  : float  Maximum jerk (m/s^3).
    n_points  : int    Number of time samples for interpolation.
    """
    d     = 5.5
    deg_1 = math.radians(90)   # north
    deg_2 = math.radians(180)  # west

    start = [2.5,     -d,    deg_1]
    end   = [5 + d,    2.5,  deg_2]
    kappa = [0, 0, 0, 0]
    n = round(math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2))
    seg = eta3_path_segment(start, end, eta=[n, n, 0, 0, 0, 0], kappa=kappa)

    traj  = eta3_trajectory([seg], max_vel=max_vel,
                            max_accel=max_accel, max_jerk=max_jerk)
    times = np.linspace(0, traj.total_time, n_points)
    state = np.column_stack([traj.calc_traj_point(t) for t in times])  # (5, N)

    # --- Figure 1: trajectory coloured by speed ---
    fig1, ax1 = plt.subplots()
    x, y = state[0], state[1]
    pts  = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc   = LineCollection(segs, cmap="inferno", linewidth=2)
    lc.set_array(state[3])
    ax1.add_collection(lc)
    ax1.set_xlim(x.min() - 1, x.max() + 1)
    ax1.set_ylim(y.min() - 1, y.max() + 1)
    cb = fig1.colorbar(lc, ax=ax1)
    cb.set_label("velocity (m/s)")
    ax1.set_title("Trajectory — coloured by speed")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    fig1.tight_layout()

    # --- Figure 2: velocity and angular velocity profiles ---
    fig2, ax_v = plt.subplots()
    ax_v.plot(times, state[3], "b-", label="linear velocity")
    ax_v.set_xlabel("time (s)")
    ax_v.set_ylabel("velocity (m/s)", color="b")
    ax_v.tick_params("y", colors="b")
    ax_v.set_title("Control — velocity and angular velocity")
    ax_w = ax_v.twinx()
    ax_w.plot(times, state[4], "r-", label="angular velocity")
    ax_w.set_ylabel("angular velocity (rad/s)", color="r")
    ax_w.tick_params("y", colors="r")
    fig2.tight_layout()

    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    run_capstone_scenario()

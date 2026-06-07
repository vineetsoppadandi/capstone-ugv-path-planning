"""
eta3_spline_path.py
-------------------
Core eta^3 (eta-cubic) spline primitives used by both the dynamic path planner
and the trajectory planner.

The eta^3 spline is a 7th-degree polynomial parameterisation that guarantees
C2 continuity (position, tangent, and curvature match) at segment boundaries,
making it suitable for wheeled and aerial robot motion planning.

References
----------
- Nagy & Kelly, "Reactive Nonholonomic Trajectory Generation via Parametric
  Optimal Control", IJRR 2001.
- Bevilacqua & Dinius, eta^3-Splines for Smooth Path Generation of Wheeled
  Mobile Robots, IEEE T-RO 2008.
  https://ieeexplore.ieee.org/document/4339545/
- PythonRobotics (Atsushi Sakai): original reference implementation
  https://github.com/AtsushiSakai/PythonRobotics
"""

import numpy as np
from scipy.integrate import quad


class eta3_path:
    """
    A continuous path composed of one or more eta^3 path segments.

    Parameters
    ----------
    segments : list of eta3_path_segment
        Ordered list of segments.  Each segment must begin at the previous
        segment's end pose (continuity enforced on construction).
    """

    def __init__(self, segments):
        if not (isinstance(segments, list) and isinstance(segments[0], eta3_path_segment)):
            raise TypeError("segments must be a non-empty list of eta3_path_segment instances")
        for prev, curr in zip(segments[:-1], segments[1:]):
            if not np.array_equal(prev.end_pose, curr.start_pose):
                raise ValueError("Segment end/start poses do not match — path is not continuous")
        self.segments = segments

    def calc_path_point(self, u):
        """
        Evaluate the path at normalised parameter u in [0, len(segments)].

        Parameters
        ----------
        u : float
            Normalised parameter along the whole path.

        Returns
        -------
        np.ndarray, shape (2,)
            (x, y) position.
        """
        if not (0 <= u <= len(self.segments)):
            raise ValueError(f"u={u} is outside [0, {len(self.segments)}]")
        if np.isclose(u, len(self.segments)):
            seg_idx, u_local = len(self.segments) - 1, 1.0
        else:
            seg_idx = int(np.floor(u))
            u_local = u - seg_idx
        return self.segments[seg_idx].calc_point(u_local)


class eta3_path_segment:
    """
    A single eta^3 polynomial path segment connecting two robot poses.

    The segment is a 7th-degree polynomial in parameter u in [0, 1].
    Boundary conditions at u=0 (start) and u=1 (end) set position, heading,
    curvature, and curvature derivative via the eta and kappa vectors.

    Parameters
    ----------
    start_pose : array-like, shape (3,)
        Starting pose [x, y, theta] in metres / radians.
    end_pose : array-like, shape (3,)
        Ending pose [x, y, theta] in metres / radians.
    eta : array-like, shape (6,), optional
        Shaping parameters [eta1A, eta1B, eta2A, eta2B, eta3A, eta3B].
        Controls path shape (larger values → wider curves).  Defaults to zeros.
    kappa : array-like, shape (4,), optional
        Curvature boundary conditions [kappa_A, kappad_A, kappa_B, kappad_B].
        Defaults to zeros (straight-line boundary curvature).
    """

    def __init__(self, start_pose, end_pose, eta=None, kappa=None):
        if len(start_pose) != 3 or len(end_pose) != 3:
            raise ValueError("start_pose and end_pose must each have 3 elements [x, y, theta]")

        self.start_pose = list(start_pose)
        self.end_pose   = list(end_pose)

        eta   = list(eta)   if eta   is not None else [0.0] * 6
        kappa = list(kappa) if kappa is not None else [0.0] * 4

        if len(eta) != 6:
            raise ValueError("eta must have 6 elements")
        if len(kappa) != 4:
            raise ValueError("kappa must have 4 elements")

        ca, sa = np.cos(start_pose[2]), np.sin(start_pose[2])
        cb, sb = np.cos(end_pose[2]),   np.sin(end_pose[2])
        dx = end_pose[0] - start_pose[0]
        dy = end_pose[1] - start_pose[1]

        # 2 x 8 coefficient matrix (row 0 = x, row 1 = y)
        c = np.zeros((2, 8))

        # u^0 — position at start
        c[0, 0] = start_pose[0]
        c[1, 0] = start_pose[1]

        # u^1 — tangent direction scaled by eta[0]
        c[0, 1] = eta[0] * ca
        c[1, 1] = eta[0] * sa

        # u^2
        c[0, 2] = 0.5 * eta[2] * ca - 0.5 * eta[0]**2 * kappa[0] * sa
        c[1, 2] = 0.5 * eta[2] * sa + 0.5 * eta[0]**2 * kappa[0] * ca

        # u^3
        c[0, 3] = (eta[4]*ca - (eta[0]**3*kappa[1] + 3*eta[0]*eta[2]*kappa[0])*sa) / 6
        c[1, 3] = (eta[4]*sa + (eta[0]**3*kappa[1] + 3*eta[0]*eta[2]*kappa[0])*ca) / 6

        # u^4
        c[0, 4] = (35*dx
                   - (20*eta[0] + 5*eta[2] + 2/3*eta[4])*ca
                   + (5*eta[0]**2*kappa[0] + 2/3*eta[0]**3*kappa[1] + 2*eta[0]*eta[2]*kappa[0])*sa
                   - (15*eta[1] - 5/2*eta[3] + 1/6*eta[5])*cb
                   - (5/2*eta[1]**2*kappa[2] - 1/6*eta[1]**3*kappa[3] - 0.5*eta[1]*eta[3]*kappa[2])*sb)
        c[1, 4] = (35*dy
                   - (20*eta[0] + 5*eta[2] + 2/3*eta[4])*sa
                   - (5*eta[0]**2*kappa[0] + 2/3*eta[0]**3*kappa[1] + 2*eta[0]*eta[2]*kappa[0])*ca
                   - (15*eta[1] - 5/2*eta[3] + 1/6*eta[5])*sb
                   + (5/2*eta[1]**2*kappa[2] - 1/6*eta[1]**3*kappa[3] - 0.5*eta[1]*eta[3]*kappa[2])*cb)

        # u^5
        c[0, 5] = (-84*dx
                   + (45*eta[0] + 10*eta[2] + eta[4])*ca
                   - (10*eta[0]**2*kappa[0] + eta[0]**3*kappa[1] + 3*eta[0]*eta[2]*kappa[0])*sa
                   + (39*eta[1] - 7*eta[3] + 0.5*eta[5])*cb
                   + (7*eta[1]**2*kappa[2] - 0.5*eta[1]**3*kappa[3] - 1.5*eta[1]*eta[3]*kappa[2])*sb)
        c[1, 5] = (-84*dy
                   + (45*eta[0] + 10*eta[2] + eta[4])*sa
                   + (10*eta[0]**2*kappa[0] + eta[0]**3*kappa[1] + 3*eta[0]*eta[2]*kappa[0])*ca
                   + (39*eta[1] - 7*eta[3] + 0.5*eta[5])*sb
                   - (7*eta[1]**2*kappa[2] - 0.5*eta[1]**3*kappa[3] - 1.5*eta[1]*eta[3]*kappa[2])*cb)

        # u^6
        c[0, 6] = (70*dx
                   - (36*eta[0] + 7.5*eta[2] + 2/3*eta[4])*ca
                   + (7.5*eta[0]**2*kappa[0] + 2/3*eta[0]**3*kappa[1] + 2*eta[0]*eta[2]*kappa[0])*sa
                   - (34*eta[1] - 6.5*eta[3] + 0.5*eta[5])*cb
                   - (6.5*eta[1]**2*kappa[2] - 0.5*eta[1]**3*kappa[3] - 1.5*eta[1]*eta[3]*kappa[2])*sb)
        c[1, 6] = (70*dy
                   - (36*eta[0] + 7.5*eta[2] + 2/3*eta[4])*sa
                   - (7.5*eta[0]**2*kappa[0] + 2/3*eta[0]**3*kappa[1] + 2*eta[0]*eta[2]*kappa[0])*ca
                   - (34*eta[1] - 6.5*eta[3] + 0.5*eta[5])*sb
                   + (6.5*eta[1]**2*kappa[2] - 0.5*eta[1]**3*kappa[3] - 1.5*eta[1]*eta[3]*kappa[2])*cb)

        # u^7
        c[0, 7] = (-20*dx
                   + (10*eta[0] + 2*eta[2] + 1/6*eta[4])*ca
                   - (2*eta[0]**2*kappa[0] + 1/6*eta[0]**3*kappa[1] + 0.5*eta[0]*eta[2]*kappa[0])*sa
                   + (10*eta[1] - 2*eta[3] + 1/6*eta[5])*cb
                   + (2*eta[1]**2*kappa[2] - 1/6*eta[1]**3*kappa[3] - 0.5*eta[1]*eta[3]*kappa[2])*sb)
        c[1, 7] = (-20*dy
                   + (10*eta[0] + 2*eta[2] + 1/6*eta[4])*sa
                   + (2*eta[0]**2*kappa[0] + 1/6*eta[0]**3*kappa[1] + 0.5*eta[0]*eta[2]*kappa[0])*ca
                   + (10*eta[1] - 2*eta[3] + 1/6*eta[5])*sb
                   - (2*eta[1]**2*kappa[2] - 1/6*eta[1]**3*kappa[3] - 0.5*eta[1]*eta[3]*kappa[2])*cb)

        self.coeffs = c
        self.s_dot = lambda u: max(
            np.linalg.norm(c[:, 1:].dot(
                [1, 2*u, 3*u**2, 4*u**3, 5*u**4, 6*u**5, 7*u**6]
            )), 1e-6
        )
        self.f_length = lambda ue: quad(self.s_dot, 0, ue)
        self.segment_length = self.f_length(1)[0]

    def calc_point(self, u):
        """
        Evaluate (x, y) position at parameter u in [0, 1].
        """
        if not (0 <= u <= 1):
            raise ValueError(f"u={u} is outside [0, 1]")
        return self.coeffs.dot([1, u, u**2, u**3, u**4, u**5, u**6, u**7])

    def calc_deriv(self, u, order=1):
        """
        Evaluate the 1st or 2nd derivative of (x, y) at parameter u.
        """
        if not (0 <= u <= 1):
            raise ValueError(f"u={u} is outside [0, 1]")
        if order == 1:
            return self.coeffs[:, 1:].dot(
                [1, 2*u, 3*u**2, 4*u**3, 5*u**4, 6*u**5, 7*u**6]
            )
        if order == 2:
            return self.coeffs[:, 2:].dot(
                [2, 6*u, 12*u**2, 20*u**3, 30*u**4, 42*u**5]
            )
        raise ValueError("order must be 1 or 2")

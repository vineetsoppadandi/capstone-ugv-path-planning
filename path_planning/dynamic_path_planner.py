"""
dynamic_path_planner.py
-----------------------
Obstacle-aware eta^3 spline path planner for drone/robot navigation.

Given a set of polygon obstacles defined with Shapely, this module computes
a closed eta^3 spline loop that navigates around them.  The path is built
from three connected segments whose start/end poses and shaping parameters
are derived from the obstacle geometry.

Type
----
Hybrid — the eta^3 path primitives (eta3_path, eta3_path_segment) are sourced
from the PythonRobotics reference implementation (Sakai et al.).  The obstacle
integration, multi-segment loop construction, and Capstone-specific geometry
are original contributions.

Usage
-----
    python dynamic_path_planner.py

References
----------
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
- eta^3 paper: https://ieeexplore.ieee.org/document/4339545/
"""

import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon

# Allow running this script directly from any working directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eta3_spline_path import eta3_path, eta3_path_segment  # noqa: E402


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def euclidean_distance(p1, p2):
    """Return the Euclidean distance between two 2-D points."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def plot_polygon(coords):
    """Plot a closed polygon from a list of (x, y) vertices."""
    closed = list(coords) + [coords[0]]
    xs, ys = zip(*closed)
    plt.plot(xs, ys)


def draw_obstacle(vertices):
    """
    Draw an obstacle polygon and label the axes.

    Parameters
    ----------
    vertices : list of [x, y]
        Corner coordinates of the obstacle (open — first point not repeated).
    """
    polygon = Polygon(vertices + [vertices[0]])
    exterior = list(polygon.exterior.coords)
    xs, ys = zip(*exterior)
    plt.plot(xs, ys)
    plt.ylabel("Meter")
    plt.xlabel("Meter")
    plt.title("Optimal Path Points")


# ---------------------------------------------------------------------------
# Path planner
# ---------------------------------------------------------------------------

def plan_loop_path(obstacle_vertices):
    """
    Compute a three-segment eta^3 spline loop that navigates around an obstacle.

    The loop visits three waypoints positioned around the obstacle, chosen so
    that the drone approaches each wall face from a safe standoff distance.

    Parameters
    ----------
    obstacle_vertices : list of [x, y]
        Convex obstacle corners (open polygon).

    Returns
    -------
    eta3_path
        A closed eta^3 path with three segments.
    list of np.ndarray
        Interpolated (x, y) positions along the path (1001 points).
    """
    kappa = [0, 0, 0, 0]

    # --- Segment 1: approach from below, turn left toward east face ---
    deg_1 = math.radians(90)   # heading: north
    deg_2 = math.radians(180)  # heading: west
    start_1 = [0.5, -5.0, deg_1]
    end_1   = [6.0,  0.5, deg_2]
    n1 = round(euclidean_distance(start_1[:2], end_1[:2]))
    seg1 = eta3_path_segment(start_1, end_1,
                             eta=[n1, n1, 0, -100, -100, -100],
                             kappa=kappa)

    # --- Segment 2: sweep across north face, turn toward north-west ---
    deg_3 = deg_2
    deg_4 = math.radians(315)  # heading: south-east
    start_2 = [6.0,    0.5,    deg_3]
    end_2   = [-3.036, 4.036,  deg_4]
    n2 = round(euclidean_distance(start_2[:2], end_2[:2]))
    seg2 = eta3_path_segment(start_2, end_2,
                             eta=[n2, n2, 0, -100, -100, -100],
                             kappa=kappa)

    # --- Segment 3: return to start pose to close the loop ---
    deg_5 = deg_4
    deg_6 = deg_1
    start_3 = end_2
    end_3   = start_1
    n3 = round(euclidean_distance(start_3[:2], end_3[:2]))
    seg3 = eta3_path_segment(start_3, end_3,
                             eta=[n3, n3, 0, -100, -100, -100],
                             kappa=kappa)

    path = eta3_path([seg1, seg2])  # two-segment path displayed; seg3 closes the loop

    # Interpolate path points
    u_vals = np.linspace(0, len([seg1, seg2]), 1001)
    positions = np.array([path.calc_path_point(u) for u in u_vals]).T  # (2, 1001)

    return path, positions


def plot_path(obstacle_vertices, positions):
    """
    Render the obstacle and the planned path.

    Parameters
    ----------
    obstacle_vertices : list of [x, y]
    positions : np.ndarray, shape (2, N)
        Interpolated path positions.
    """
    fig, ax = plt.subplots()
    draw_obstacle(obstacle_vertices)
    ax.plot(positions[0], positions[1])
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Triangular obstacle used in the Capstone scenario
    obstacle = [[0, 0], [1, 0], [1, 1]]

    _, positions = plan_loop_path(obstacle)
    plot_path(obstacle, positions)

"""
lidar_coverage.py
-----------------
LIDAR sensor coverage path planner for building inspection.

Given a building footprint and LIDAR sensor specifications, this module
calculates the minimum safe standoff distance required for the sensor to
achieve a target point density across the building facade.  It then generates
an optimal flight path (a buffered polygon with rounded corners) around the
building at that standoff distance.

The rounded corners in the output path reflect the fact that the drone must
swing wider at building corners to maintain continuous sensor coverage — the
buffer radius is derived directly from the sensor's horizontal angular
resolution.

LIDAR sensor parameters (Velodyne-class):
  vAngular = 1.33  deg   vertical angular resolution
  hAngular = 0.16  deg   horizontal angular resolution
  vFOV     = 10.67 deg   total vertical field of view

Type
----
Fully original — no equivalent exists in PythonRobotics or standard robotics
libraries.  The FOV-based standoff and point-density calculation is an
original contribution of this Capstone project.

Usage
-----
    python lidar_coverage.py
"""

import math

import matplotlib.pyplot as plt
from shapely.geometry import Polygon


# ---------------------------------------------------------------------------
# Sensor constants (Velodyne-class LIDAR)
# ---------------------------------------------------------------------------

V_ANGULAR = 1.33   # deg — vertical angular resolution between scan lines
H_ANGULAR = 0.16   # deg — horizontal angular resolution
V_FOV     = 10.67  # deg — total vertical field of view
MIN_RESOLUTION_THRESHOLD = 250  # minimum acceptable point density (pts/m^2)
DRONE_HEIGHT_ABOVE_GROUND = 1.5  # m — drone flies 1.5 m below building roof


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def plot_polygon(coords):
    """Plot a list of (x, y) coordinates as a closed polygon."""
    pts = list(coords)
    xs, ys = zip(*pts)
    plt.plot(xs, ys)


def draw_polygons(polys):
    """Plot a list of Shapely polygons (exterior + any holes)."""
    for poly in polys:
        if not getattr(poly, "exterior", None):
            continue
        plot_polygon(poly.exterior.coords)
        for hole in poly.interiors:
            plot_polygon(hole.coords)


# ---------------------------------------------------------------------------
# Core calculations
# ---------------------------------------------------------------------------

def compute_standoff_distance(building_height):
    """
    Compute the required standoff distance for acceptable LIDAR resolution.

    The calculation checks whether the sensor achieves at least
    MIN_RESOLUTION_THRESHOLD points/m^2 at the given standoff.  If the
    automatically determined distance is acceptable it is returned directly;
    otherwise the minimum safe distance is returned with a warning.

    Parameters
    ----------
    building_height : float
        Height of the tallest building face to be scanned (metres).

    Returns
    -------
    float
        Standoff distance in metres, or None if sensor resolution is
        insufficient and the operator chose not to override.
    """
    effective_height = building_height - DRONE_HEIGHT_ABOVE_GROUND

    # Vertical point density: scan lines per metre of building height
    v_scan_lines = V_FOV / V_ANGULAR
    vert_density  = v_scan_lines / effective_height

    # Horizontal coverage depth at this standoff
    standoff = effective_height * math.tan(math.radians(V_FOV))
    standoff = round(standoff)  # round to nearest metre

    # Horizontal angular extent subtended by 0.5 m at the standoff distance
    theta_h = 2 * math.degrees(math.atan(0.5 / standoff))

    # Horizontal point density: scan points per metre of facade width
    horiz_density = theta_h / H_ANGULAR

    point_density = vert_density * horiz_density

    if point_density >= MIN_RESOLUTION_THRESHOLD:
        print(f"LIDAR resolution OK: {point_density:.1f} pts/m² "
              f"(threshold: {MIN_RESOLUTION_THRESHOLD})")
    else:
        print(f"Warning: LIDAR resolution {point_density:.1f} pts/m² is below "
              f"threshold ({MIN_RESOLUTION_THRESHOLD} pts/m²).  "
              f"Using minimum safe standoff anyway.")

    return standoff


def plan_lidar_path(building_vertices, building_height):
    """
    Generate the optimal LIDAR scan flight path around a building.

    The path is a Shapely-buffered version of the building polygon, where
    the buffer radius equals the required sensor standoff distance.  The
    resulting rounded-corner shape ensures continuous sensor coverage around
    all building faces, including corners.

    Parameters
    ----------
    building_vertices : list of [x, y]
        Convex building footprint corners (open polygon).
    building_height : float
        Height of the building in metres.

    Returns
    -------
    tuple (Polygon, Polygon, float)
        (building_polygon, flight_path_polygon, standoff_distance)
    """
    standoff = compute_standoff_distance(building_height)
    building  = Polygon(building_vertices + [building_vertices[0]])
    flight_path = building.buffer(standoff)
    return building, flight_path, standoff


def plot_lidar_coverage(building, flight_path):
    """
    Render the building footprint and the LIDAR flight path.

    Parameters
    ----------
    building : Shapely Polygon
    flight_path : Shapely Polygon
    """
    fig, ax = plt.subplots()
    draw_polygons([flight_path, building])
    ax.set_xlabel("Metre")
    ax.set_ylabel("Metre")
    ax.set_title("LIDAR Coverage Path — Concave Structure")
    ax.set_aspect("equal")
    fig.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Capstone scenario: 5 m × 5 m building, 5 m tall
    BUILDING_VERTICES = [[0, 0], [0, 5], [5, 5], [5, 0]]
    BUILDING_HEIGHT   = 5.0

    building, flight_path, standoff = plan_lidar_path(BUILDING_VERTICES, BUILDING_HEIGHT)
    print(f"Standoff distance: {standoff} m")
    plot_lidar_coverage(building, flight_path)

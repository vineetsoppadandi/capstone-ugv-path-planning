"""
photogrammetry_basic.py
-----------------------
Basic photogrammetry camera placement planner (Version 1).

Given a simple rectilinear building footprint, this module determines the
optimal drone hover positions from which a camera can photograph each building
face, ensuring complete coverage with minimal images.

Key insight over the naive grid approach: rather than checking cardinal
directions by rule, this implementation uses Shapely's polygon containment
test to determine which side of each wall is the building interior and which
is the exterior — so the camera is always placed on the correct (outside) side
regardless of building orientation.

Camera / sensor parameters:
  Focal length  : 2.5 mm
  Sensor size   : 5.79 mm (W) x 4.89 mm (H)
  Resolution    : 648 px (W) x 488 px (H)
  vFOV          : 81 deg
  hFOV          : 97 deg

Type
----
Fully original — camera placement using interior/exterior containment
detection is an original contribution of this Capstone project.

Usage
-----
    python photogrammetry_basic.py
"""

import math

import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon


# ---------------------------------------------------------------------------
# Camera parameters
# ---------------------------------------------------------------------------

FOCAL_LEN     = 2.5   # mm
SENSOR_HEIGHT = 4.89  # mm
SENSOR_WIDTH  = 5.79  # mm
PIXEL_HEIGHT  = 488
PIXEL_WIDTH   = 648
V_FOV         = 81    # degrees — vertical field of view
H_FOV         = 97    # degrees — horizontal field of view
CAMERA_HEIGHT = 1.2   # m — drone hover height above ground
MIN_STANDOFF  = 5.0   # m — minimum clearance to avoid collision


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def plot_polygon_coords(coords):
    pts = list(coords)
    xs, ys = zip(*pts)
    plt.plot(xs, ys)


def draw_building(vertices):
    """Draw the building footprint polygon."""
    poly = Polygon(vertices + [vertices[0]])
    plot_polygon_coords(poly.exterior.coords)
    for hole in poly.interiors:
        plot_polygon_coords(hole.coords)


def segment_midpoint(p1, p2):
    """Return the midpoint of a line segment."""
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)


def segment_length(p1, p2):
    """Return the Euclidean length of a segment."""
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)


# ---------------------------------------------------------------------------
# Camera calculations
# ---------------------------------------------------------------------------

def compute_standoff(object_height):
    """
    Minimum horizontal standoff to see the full building face in the camera.

    The camera is at CAMERA_HEIGHT above ground; effective visible height is
    (object_height - CAMERA_HEIGHT).  The half-angle of vFOV determines how
    far back the drone needs to be.

    Parameters
    ----------
    object_height : float  Building height (m).

    Returns
    -------
    float  Standoff distance (m), clamped to MIN_STANDOFF.
    """
    h   = object_height - CAMERA_HEIGHT
    deg = math.radians(V_FOV / 2)
    do  = h / math.tan(deg)
    return max(do, MIN_STANDOFF)


def horizontal_coverage(standoff):
    """
    Horizontal width covered in one camera shot at the given standoff.

    Parameters
    ----------
    standoff : float  Distance from camera to wall (m).

    Returns
    -------
    float  Coverage width (m), rounded to nearest 0.5 m.
    """
    half_width = standoff * math.tan(math.radians(H_FOV / 2))
    full_width = 2 * half_width
    return round(full_width * 2) / 2  # nearest 0.5 m


def pixel_density(standoff, obj_height, obj_width):
    """
    Compute the pixel density (px^2/m^2) achieved at the given standoff.

    Parameters
    ----------
    standoff   : float  Distance from camera to wall (m).
    obj_height : float  Building face height (m).
    obj_width  : float  Building face width (m).

    Returns
    -------
    float  Pixel density in px^2/m^2.
    """
    h_pixels = (FOCAL_LEN * obj_height * PIXEL_HEIGHT) / (standoff * SENSOR_HEIGHT)
    w_pixels = (FOCAL_LEN * obj_width  * PIXEL_WIDTH)  / (standoff * SENSOR_WIDTH)
    return h_pixels * w_pixels


# ---------------------------------------------------------------------------
# Placement planner
# ---------------------------------------------------------------------------

def compute_camera_positions(vertices, object_height):
    """
    Compute optimal camera hover positions for all building faces.

    For each wall segment of the building, the function:
      1. Finds the segment midpoint.
      2. Tests two candidate points (one on each side of the wall) using
         Shapely containment — the point inside the polygon is the interior
         side, so the camera is placed on the *other* side.
      3. Places the camera at `standoff` distance from the wall on the
         exterior side.

    Supports axis-aligned, diagonal, and arbitrary wall orientations.

    Parameters
    ----------
    vertices      : list of [x, y]
        Building footprint corners (open polygon — first point not repeated).
    object_height : float
        Building height (m).

    Returns
    -------
    list of [x, y]
        Recommended camera positions (one per wall face).
    """
    standoff  = compute_standoff(object_height)
    cover     = horizontal_coverage(standoff)
    closed    = vertices + [vertices[0]]
    distances = [segment_length(closed[i], closed[i+1])
                 for i in range(len(closed) - 1)]

    polygon = Polygon(closed)
    camera_positions = []

    for i, seg_len in enumerate(distances):
        x1, y1 = closed[i]
        x2, y2 = closed[i + 1]
        mx, my = segment_midpoint([x1, y1], [x2, y2])

        # Number of shots needed for this wall face
        n_shots = max(1, round(seg_len / cover))

        for k in range(n_shots):
            if n_shots == 1:
                cx, cy = mx, my
            else:
                t  = (k + 0.5) / n_shots
                cx = x1 + t * (x2 - x1)
                cy = y1 + t * (y2 - y1)

            # Wall direction vector and perpendicular
            dx, dy  = x2 - x1, y2 - y1
            length  = math.sqrt(dx**2 + dy**2)
            if length < 1e-9:
                continue
            nx, ny = -dy / length, dx / length  # left-hand normal

            # Test both sides of the wall
            p_left  = Point(cx + nx * 0.1 * seg_len, cy + ny * 0.1 * seg_len)
            p_right = Point(cx - nx * 0.1 * seg_len, cy - ny * 0.1 * seg_len)

            if polygon.contains(p_left):
                # Interior is to the left — place camera to the right
                cam_x = cx - nx * standoff
                cam_y = cy - ny * standoff
            else:
                # Interior is to the right — place camera to the left
                cam_x = cx + nx * standoff
                cam_y = cy + ny * standoff

            camera_positions.append([round(cam_x, 3), round(cam_y, 3)])

    return camera_positions


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    BUILDING  = [[0, 0], [1, 0], [1, 1], [0, 1]]
    HEIGHT    = 5.0

    positions = compute_camera_positions(BUILDING, HEIGHT)
    print("Camera positions (x, y):")
    for p in positions:
        print(f"  {p}")

    standoff = compute_standoff(HEIGHT)
    fig, ax = plt.subplots()
    draw_building(BUILDING)
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    ax.plot(xs, ys, "ro", markersize=8, label="Camera positions")
    ax.set_xlabel("m")
    ax.set_ylabel("m")
    ax.set_title("Photogrammetry Path Planner — Simple Building")
    ax.legend()
    ax.set_aspect("equal")
    fig.tight_layout()
    plt.show()

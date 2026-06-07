"""
photogrammetry_advanced.py
--------------------------
Advanced photogrammetry camera placement planner (Version 2).

Extends the basic planner to handle complex, non-convex (L-shaped, U-shaped,
or otherwise concave) building footprints.  Each wall face of a multi-segment
perimeter receives one or more camera positions depending on its length
relative to the horizontal coverage width.

Camera / sensor parameters:
  Focal length  : 2.5 mm
  Sensor size   : 5.79 mm (W) x 4.89 mm (H)
  Resolution    : 648 px (W) x 488 px (H)
  vFOV          : 81 deg
  hFOV          : 97 deg

Type
----
Fully original — extension of the Version 1 planner to support concave/complex
building footprints.  An original contribution of this Capstone project.

Usage
-----
    python photogrammetry_advanced.py
"""

import math

import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon


# ---------------------------------------------------------------------------
# Camera parameters
# ---------------------------------------------------------------------------

FOCAL_LEN     = 2.5
SENSOR_HEIGHT = 4.89
SENSOR_WIDTH  = 5.79
PIXEL_HEIGHT  = 488
PIXEL_WIDTH   = 648
V_FOV         = 81
H_FOV         = 97
CAMERA_HEIGHT = 1.2
MIN_STANDOFF  = 5.0


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


def segment_length(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)


# ---------------------------------------------------------------------------
# Camera calculations  (same physics as v1)
# ---------------------------------------------------------------------------

def compute_standoff(object_height):
    """Minimum standoff to frame the full building face; clamped to MIN_STANDOFF."""
    h   = object_height - CAMERA_HEIGHT
    deg = math.radians(V_FOV / 2)
    do  = h / math.tan(deg)
    return max(do, MIN_STANDOFF)


def horizontal_coverage(standoff):
    """Full horizontal width covered in one shot, rounded to 0.5 m."""
    full_width = 2 * standoff * math.tan(math.radians(H_FOV / 2))
    return round(full_width * 2) / 2


def pixel_density(standoff, obj_height, obj_width):
    """Pixel density in px^2/m^2 at the given standoff."""
    hp = (FOCAL_LEN * obj_height * PIXEL_HEIGHT) / (standoff * SENSOR_HEIGHT)
    wp = (FOCAL_LEN * obj_width  * PIXEL_WIDTH)  / (standoff * SENSOR_WIDTH)
    return hp * wp


# ---------------------------------------------------------------------------
# Placement planner
# ---------------------------------------------------------------------------

def compute_camera_positions(vertices, object_height):
    """
    Compute optimal camera positions for a complex (potentially concave) building.

    Algorithm
    ---------
    For each wall segment:
      1. Determine how many shots are required (segment_length / coverage_width).
      2. Evenly distribute shot centres along the segment.
      3. Use Shapely containment to identify the exterior side.
      4. Place the camera at `standoff` distance on the exterior side.

    Parameters
    ----------
    vertices      : list of [x, y]
        Building perimeter vertices (open polygon).  Can be concave.
    object_height : float
        Building height (m).

    Returns
    -------
    list of [x, y]
        Camera hover positions — one or more per wall face.
    """
    standoff = compute_standoff(object_height)
    cover    = horizontal_coverage(standoff)
    closed   = vertices + [vertices[0]]
    polygon  = Polygon(closed)

    camera_positions = []

    for i in range(len(closed) - 1):
        x1, y1 = closed[i]
        x2, y2 = closed[i + 1]
        seg_len = segment_length([x1, y1], [x2, y2])

        if seg_len < 1e-6:
            continue  # skip degenerate segments

        # Wall direction and left-hand perpendicular normal
        dx, dy = x2 - x1, y2 - y1
        nx, ny = -dy / seg_len, dx / seg_len

        n_shots = max(1, round(seg_len / cover))

        for k in range(n_shots):
            t  = (k + 0.5) / n_shots
            cx = x1 + t * dx
            cy = y1 + t * dy

            # Containment test: which side is interior?
            p_left  = Point(cx + nx * 0.1 * seg_len, cy + ny * 0.1 * seg_len)
            p_right = Point(cx - nx * 0.1 * seg_len, cy - ny * 0.1 * seg_len)

            if polygon.contains(p_left):
                cam_x = cx - nx * standoff
                cam_y = cy - ny * standoff
            else:
                cam_x = cx + nx * standoff
                cam_y = cy + ny * standoff

            camera_positions.append([round(cam_x, 3), round(cam_y, 3)])

    return camera_positions


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Capstone scenario: L-shaped building (10 m x 10 m with south-east notch)
    L_SHAPED_BUILDING = [
        [0,  0],
        [10, 0],
        [10, 5],
        [5,  5],
        [5,  10],
        [0,  10],
    ]
    HEIGHT = 5.0

    positions = compute_camera_positions(L_SHAPED_BUILDING, HEIGHT)
    print("Camera positions (x, y):")
    for p in positions:
        print(f"  {p}")

    fig, ax = plt.subplots()
    draw_building(L_SHAPED_BUILDING)
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    ax.plot(xs, ys, "ro", markersize=8, label="Camera positions")
    ax.set_xlabel("m")
    ax.set_ylabel("m")
    ax.set_title("Photogrammetry Path Planner — L-shaped Building")
    ax.legend()
    ax.set_aspect("equal")
    fig.tight_layout()
    plt.show()

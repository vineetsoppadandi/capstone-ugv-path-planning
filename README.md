# Capstone: Drone Survey Path Planning

Autonomous drone path planning and sensor coverage algorithms developed as part of a Masters Capstone project at the **University of Melbourne**.  The project investigates how a drone can be guided to survey a building autonomously — computing both the geometric flight path and the optimal sensor positions for LIDAR and photogrammetry capture.

---

## Project Overview

Surveying a building with a drone requires solving two distinct problems:

1. **Path planning** — how does the drone move smoothly from A to B while respecting curvature constraints and avoiding obstacles?
2. **Sensor coverage** — where should the drone hover to ensure every face of the building is captured within the sensor's field of view?

This project addresses both, producing five runnable Python modules across two packages.

---

## Repository Structure

```
capstone-drone-survey-planning/
├── path_planning/
│   ├── eta3_spline_path.py          # Shared eta^3 spline primitives
│   ├── dynamic_path_planner.py      # Obstacle-aware path planner
│   └── trajectory_planner.py        # Path + S-curve velocity profile
├── sensor_coverage/
│   ├── lidar_coverage.py            # LIDAR scan path planner
│   ├── photogrammetry_basic.py      # Camera placement — simple building
│   └── photogrammetry_advanced.py   # Camera placement — complex building
├── requirements.txt
└── README.md
```

---

## Modules

### `path_planning/eta3_spline_path.py`
**Shared module — eta^3 spline primitives**

Provides the `eta3_path` and `eta3_path_segment` classes that underpin both path planning scripts.  An eta^3 (eta-cubic) spline is a 7th-degree polynomial parameterisation that guarantees C² continuity at segment boundaries — meaning position, heading, curvature, and curvature derivative all match seamlessly where segments join.  This makes it well-suited for robot motion planning where abrupt changes in curvature would cause wheel slip or aerodynamic instability.

> **Origin:** Adapted from the PythonRobotics reference implementation (Sakai et al.).

---

### `path_planning/dynamic_path_planner.py`
**Obstacle-aware eta^3 path planner**

Computes a three-segment eta^3 spline loop that navigates a drone around a polygon obstacle.  The obstacle geometry is defined using Shapely, and the path's shaping parameters (eta values) are derived automatically from the Euclidean distances between waypoints.

**Output:** A matplotlib plot showing the closed flight path around the obstacle.

> **Origin:** Hybrid — eta^3 primitives from PythonRobotics; obstacle integration, multi-segment loop construction, and Capstone geometry are original.

---

### `path_planning/trajectory_planner.py`
**eta^3 trajectory with jerk-limited velocity profile**

Extends the geometric path with a time parameterisation.  A seven-section S-curve velocity profile is computed so the drone:
- Starts and ends at rest (v = 0, a = 0)
- Ramps up to cruising speed with bounded jerk (smooth acceleration)
- Cruises at maximum speed
- Decelerates symmetrically back to rest

**Output:** Two matplotlib figures — (1) the trajectory path coloured by speed using the `inferno` colormap, and (2) linear and angular velocity profiles over time.

> **Origin:** Hybrid — velocity profile algorithm from PythonRobotics/Dinius; Capstone-specific start/end poses and scenario setup are original.

---

### `sensor_coverage/lidar_coverage.py`
**LIDAR sensor coverage path planner**

Given a building footprint and LIDAR sensor specifications, this module:
1. Calculates the minimum standoff distance at which the sensor achieves a target point density (≥ 250 pts/m²) across the building facade.
2. Generates a Shapely-buffered flight path at that standoff, with rounded corners to maintain continuous coverage around building edges.

LIDAR sensor parameters used (Velodyne-class):

| Parameter | Value |
|-----------|-------|
| Vertical angular resolution | 1.33° |
| Horizontal angular resolution | 0.16° |
| Vertical FOV | 10.67° |

**Output:** A matplotlib plot showing the building footprint (orange) and the computed flight path (blue).

> **Origin:** Fully original.

---

### `sensor_coverage/photogrammetry_basic.py`
**Photogrammetry path planner — simple rectilinear building (v1)**

Given a rectilinear building footprint, computes optimal drone hover positions for photographing each wall face.  Uses **Shapely polygon containment testing** to determine which side of each wall is the building interior — so the camera is always correctly placed on the exterior side regardless of building orientation.

Camera parameters:

| Parameter | Value |
|-----------|-------|
| Focal length | 2.5 mm |
| Sensor | 5.79 × 4.89 mm, 648 × 488 px |
| Vertical FOV | 81° |
| Horizontal FOV | 97° |

**Output:** Camera positions printed to terminal + matplotlib plot of the building and hover points.

> **Origin:** Fully original.  The interior/exterior detection via Shapely containment is an original contribution.

---

### `sensor_coverage/photogrammetry_advanced.py`
**Photogrammetry path planner — complex building (v2)**

Extends the basic planner to support non-convex (L-shaped, U-shaped, or arbitrary concave) building footprints.  For long wall faces, multiple camera positions are computed along the face to maintain full horizontal coverage at the required standoff distance.

**Example output for the L-shaped Capstone building:**
```
Camera positions (x, y):
  [5.0, -5.0]
  [15.0, 2.5]
  [7.5, 10.0]
  [10.0, 7.5]
  [2.5, 15.0]
  [-5.0, 5.0]
```

**Output:** Camera positions printed to terminal + matplotlib plot.

> **Origin:** Fully original.

---

## Getting Started

### Prerequisites

- Python 3.8+
- [uv](https://docs.astral.sh/uv/) (recommended) **or** pip

### Install dependencies

```bash
# With pip
pip install -r requirements.txt

# With uv (auto-installs Python if needed)
uv pip install -r requirements.txt
```

### Run each script

```bash
# Path planning
python path_planning/dynamic_path_planner.py
python path_planning/trajectory_planner.py

# Sensor coverage
python sensor_coverage/lidar_coverage.py
python sensor_coverage/photogrammetry_basic.py
python sensor_coverage/photogrammetry_advanced.py
```

---

## References

- A. Sakai et al., **PythonRobotics**: A Python code collection of robotics algorithms.  
  https://github.com/AtsushiSakai/PythonRobotics

- R. Bevilacqua and T. Frazzoli, **η³-Splines for the Smooth Path Generation of Wheeled Mobile Robots**, IEEE Transactions on Robotics, 2008.  
  https://ieeexplore.ieee.org/document/4339545/

- J. Dinius, **Smooth Trajectory Generation Using Eta^3 Splines**, 2018.  
  https://jwdinius.github.io/blog/2018/eta3traj

---

## Academic Context

Developed as part of the **Capstone** subject in the Master of Engineering (Software) program at the **University of Melbourne**.  The project integrates motion planning theory (eta^3 splines) with practical sensor modelling (LIDAR resolution, camera FOV) to produce a complete autonomous building survey system.

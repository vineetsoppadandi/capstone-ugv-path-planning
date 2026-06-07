# UGV Optimal Path Planning for Optimal Data Collection

**Capstone Project — CP12-IMA-191**  
Department of Mechanical Engineering, University of Melbourne  
Final Report: 25 October 2019

| Role | Name |
|---|---|
| Student | Vineet Soppadandi (888495) |
| Academic Supervisor | Professor Ivan Marusic |
| Academic Examiner | Mr Lawrence Molloy |
| Industry Supervisor | Dr Martin Tomko |
| Client Organisation | University of Melbourne, Melbourne School of Engineering |

---

## Executive Summary

This project was inspired by data collection challenges encountered at the **Budj Bim UNESCO World Heritage Site** in western Victoria — a Gunditjmara cultural heritage site containing ancient house structures made from rocks.

When a HUSKY A200 rover was manually piloted around these structures to collect LIDAR and photogrammetry data, several critical problems emerged:

- **Inconsistent LIDAR point density** — the distance from the rover to each structure varied, causing some walls to be barely captured and others over-sampled
- **Inconsistent photogrammetry pixel density** — photos taken at varying distances produced images of the same house at vastly different scales, significantly increasing post-processing effort
- **Physical damage to the heritage site** — the rover's repeated stopping and re-orientating at waypoints left heavy tyre marks on the soft ground

This project addresses **two data-collection objectives** — one for each sensor — with autonomous path planning:

1. **LIDAR coverage path** — a constant-standoff perimeter route that maintains a fixed sensor-to-wall distance and keeps the laser perpendicular to every wall face, ensuring a consistent point density of ≥ 250 pts/m² throughout
2. **Photogrammetry coverage path** — an optimal set of capture positions where the rover stops perpendicular to each wall face to photograph it at a consistent pixel density, connected by smooth η³ (7th-order polynomial) splines so the rover never has to stop and spin to re-orient

These two objectives are implemented across four Python modules (detailed in [Algorithms](#algorithms) below), including a trajectory-planning extension that adds a jerk-limited velocity profile to the spline path.

---

## Background & Motivation

The Budj Bim heritage site has a flat, soft surface. The house structures — built from rocks — are approximately rectangular. Collecting accurate 3D data from these structures is essential for archaeological research, AI-based feature recognition, and cultural preservation.

The rover was initially operated manually with no real-time data feedback to the operator. Post-processing revealed:

- LIDAR data with heavily varying point densities between house structures despite the pilot's best efforts to maintain a consistent standoff
- Photogrammetry images of the same structure at wildly different scales (visible from the tyre marks left in the ground)
- Tyre damage to the heritage site caused by repeated stopping, spinning, and re-orienting

An autonomous path planning algorithm was identified as the solution — one that plans the route *before* deployment based on sensor specifications and building geometry, leaving no room for human error during data collection.

---

## System — HUSKY A200

The HUSKY A200 is an industrial-grade unmanned ground vehicle (UGV) running ROS on Ubuntu 16.04.

### Mapping Sensors

| Sensor | Specification |
|--------|--------------|
| **Bumblebee Stereo Camera** | Focal length 3.8 mm, HFOV 66°, VFOV 52°, ICX424 sensor, 648 × 488 px |
| **Velodyne LIDAR 32HDL** | vFOV 10.67°, hAngular 0.16°, vAngular 1.33°, 32 lasers, 700,000 pts/s, 100 m range |

### Navigation Sensors

| Sensor | Purpose |
|--------|---------|
| Wheel odometers | Real-time wheel speed and distance |
| IMU (MicroStrain 3DM-GX5-25) | Acceleration and orientation at 1000 Hz |
| GNSS RTK GPS (SwiftNav Duro) | Centimetre-level positioning at 10 Hz |

### Rover Dynamics

The HUSKY is modelled as a differential-drive robot:

```
State:      X = [x, y, θ]ᵀ
Inputs:     V = [v, ω]ᵀ  (linear and angular velocity)
Kinematics: Ẋ = [v·cos θ, v·sin θ, ω]ᵀ
```

---

## Algorithms

The two data-collection objectives map onto four Python modules:

| # | Objective / Role | Module |
|---|------------------|--------|
| 1 | LIDAR optimal coverage | `sensor_coverage/lidar_coverage.py` |
| 2 | Photogrammetry optimal coverage | `sensor_coverage/photogrammetry_basic.py`, `photogrammetry_advanced.py` |
| 3 | Smooth traversal between capture positions | `path_planning/dynamic_path_planner.py` |
| 4 | Jerk-limited motion (trajectory extension) | `path_planning/trajectory_planner.py` |

Modules 1 and 2 solve the two core objectives; modules 3 and 4 provide the smooth η³ motion that links the photogrammetry capture positions together without damaging the heritage ground.

---

### 1. LIDAR Optimal Coverage Path (`sensor_coverage/lidar_coverage.py`)

**Objective:** Maintain a consistent LIDAR point density of ≥ 250 pts/m² across all faces of the building structure.

**Key constraint:** The angle of incidence between the laser and the surface must be minimised — ideally 0° (laser return at 90° to the surface). A higher angle of incidence directly reduces point density and increases correction bias in post-processing.

**Point density formula:**

```
Point density  = (Horizontal laser points / dh) × (Vertical laser points / dv)

Horizontal laser points = θh / resolution_H
Vertical laser points   = θv / resolution_V
```

Where:
- `θv` = vertical angle subtended by the full building height at the computed standoff
- `θh` = horizontal angle subtended by 0.5 m of facade width at the standoff
- `resolution_H` = 0.16° (Velodyne horizontal angular resolution)
- `resolution_V` = 1.33° (Velodyne vertical angular resolution)

The algorithm uses **Shapely's polygon buffer** to generate the equidistant drive path at the computed standoff distance. The rounded corners in the output path keep the rover at a constant distance from the wall as it rounds each building edge, preserving continuous sensor coverage.

**Limitation:** For concave structures, the buffered path produces a sharp turning point requiring the rover to re-orient — manual control is recommended for those cases.

---

### 2. Photogrammetry Optimal Coverage Path (`sensor_coverage/photogrammetry_basic.py` and `photogrammetry_advanced.py`)

**Objective:** Compute optimal capture positions for the rover such that each building face is photographed perpendicularly with a consistent pixel density.

**Standoff distance (camera must see full building height):**

```
Distance to Object (mm) = focal_length × real_height × image_height_pixels
                          ─────────────────────────────────────────────────
                           object_height_pixels × sensor_height
```

**Number of camera positions per wall face (proportion formula):**

```
Proportion = DH / CH
```

Where `DH` is the wall face width and `CH` is the horizontal coverage width at the computed standoff. If proportion ≤ 1, one position per face is sufficient. If proportion > 1 (e.g. 1.7), the algorithm places 2 positions to ensure the full wall is captured.

**Interior/exterior detection:** The algorithm uses **Shapely point containment** to determine which side of each wall is the building interior. The camera is always placed on the exterior side, regardless of building orientation — this is the key improvement in `photogrammetry_basic.py` over simpler directional-rule approaches.

**Limitation:** The pixel density formula assumes the wall surface is perpendicular to the camera (no tilting). For curved house structures, the algorithm is less accurate. For concave structures, optimal points near the inner corner may overlap — flagged as future work.

---

### 3. η³ Spline Path Planning (`path_planning/dynamic_path_planner.py`)

**Objective:** Link the capture positions from module 2 with a smooth path that never requires the rover to stop and spin to re-orient.

A straight-line traversal between positions would force the rover to halt and rotate on the spot at each waypoint — exactly the motion that damages the soft heritage ground. The η³ spline avoids this by curving smoothly through each position with a continuously defined heading. The module demonstrates this by chaining multiple η³ segments into a closed loop around a sample obstacle (see screenshot), using the same path primitive that connects real capture positions.

The **η³ (eta-cubic) spline** is a 7th-order polynomial:

```
α(u) = a₀ + a₁u + a₂u² + a₃u³ + a₄u⁴ + a₅u⁵ + a₆u⁶ + a₇u⁷
β(u) = β₀ + β₁u + β₂u² + β₃u³ + β₄u⁴ + β₅u⁵ + β₆u⁶ + β₇u⁷
```

With 10 design parameters split between:
- **κ = [κa, κ̇a, κb, κ̇b]** — curvature and curvature derivative at start/end
- **η = [η₁, η₂, η₃, η₄, η₅, η₆]** — velocity and acceleration shaping at start/end (first 3 affect velocity; last 3 affect acceleration)

**Sub-optimal parameter selection** (used in this project):

```
η₁ = η₂ = |pa − pb|     (distance between start and end poses)
All other η and κ = 0
```

This ensures a smooth C² continuous path (matching position, heading, and curvature at segment boundaries) with no abrupt direction changes — critical for operating on soft, sensitive ground.

---

### 4. Trajectory Planner with Velocity Profile (`path_planning/trajectory_planner.py`)

**Objective:** Add a time parameterisation to the η³ geometric path, computing a smooth velocity profile so the rover starts and stops at rest without jerky motion at turning points.

A **seven-section S-curve (jerk-limited) velocity profile** is computed:

```
Sections: [max jerk up] → [max accel] → [reduce jerk] → [cruise] → [reduce jerk] → [max decel] → [max jerk down]
```

This ensures continuous velocity and acceleration throughout the trajectory — the angular velocity spike visible at the tightest turn point (where the rover transitions 90° heading) is accommodated smoothly within the profile bounds.

> **Note:** Full trajectory planning was identified during the project as the recommended next step but was discontinued due to time constraints. This implementation completes that work.

---

## Repository Structure

```
capstone-ugv-path-planning/
├── path_planning/
│   ├── eta3_spline_path.py          # η³ spline primitives (C² path segment class)
│   ├── dynamic_path_planner.py      # Multi-segment η³ loop around obstacles
│   └── trajectory_planner.py        # η³ path + jerk-limited S-curve velocity profile
├── sensor_coverage/
│   ├── lidar_coverage.py            # LIDAR standoff and buffered coverage path
│   ├── photogrammetry_basic.py      # Camera placement — simple rectilinear building
│   └── photogrammetry_advanced.py   # Camera placement — concave/L-shaped building
├── screenshots/                     # Output figures (generated by generate_screenshots.py)
├── generate_screenshots.py          # Regenerates all output figures headlessly
├── requirements.txt
└── README.md
```

---

## Output Screenshots

Figures are ordered to match the [Algorithms](#algorithms) section above.

### 1. LIDAR Coverage Path — constant-standoff perimeter with rounded corners
![LIDAR coverage](screenshots/3_lidar_coverage_1.png)

### 2a. Photogrammetry Capture Positions — simple rectilinear building
![Photogrammetry basic](screenshots/4_photogrammetry_basic_1.png)

### 2b. Photogrammetry Capture Positions — L-shaped concave building
![Photogrammetry advanced](screenshots/5_photogrammetry_advanced_1.png)

### 3. η³ Spline Path — smooth multi-segment loop around an obstacle
![Dynamic path planner](screenshots/1_dynamic_path_planner_1.png)

### 4a. Trajectory — path coloured by speed (inferno scale)
![Trajectory coloured by speed](screenshots/2_trajectory_planner_1.png)

### 4b. Trajectory — linear and angular velocity profiles over time
![Velocity profiles](screenshots/2_trajectory_planner_2.png)

---

## Getting Started

### Prerequisites

Python 3.8+ and one of:
- [uv](https://docs.astral.sh/uv/) (recommended — auto-manages Python)
- pip

### Install dependencies

```bash
pip install -r requirements.txt
```

### Run each module

```bash
# 1. LIDAR coverage
python sensor_coverage/lidar_coverage.py

# 2. Photogrammetry capture positions
python sensor_coverage/photogrammetry_basic.py
python sensor_coverage/photogrammetry_advanced.py

# 3. η³ spline path
python path_planning/dynamic_path_planner.py

# 4. Trajectory with velocity profile
python path_planning/trajectory_planner.py
```

### Regenerate screenshots

```bash
python generate_screenshots.py
```

---

## References

1. J. M. Glasgow et al., "Optimizing Information Value: Improving Rover Sensor Data Collection," *IEEE Trans. Syst., Man, Cybern.*, vol. 38, no. 3, pp. 593–601, 2008.

2. M. J. Lato et al., "Bias Correction for View-limited Lidar Scanning of Rock Outcrops for Structural Characterization," 2009.

3. M. Tomas and A. Abellan, "Rockfall detection from terrestrial short LiDAR point clouds: A clustering approach using R," *Journal of Spatial Information Science*, pp. 95–110, 2014. *(source of the 250 pts/m² threshold)*

4. J. Casper and R. Murphy, "Human–robot interactions during the robot-assisted urban search and rescue response at the World Trade Center," *IEEE Trans. Syst., Man, Cybern. B*, vol. 3, p. 367, 2003.

5. A. Takahashi et al., "Local Path Planning and Motion Control for AGV in Positioning," *Int. Workshop on Intelligent Robot and Systems*, p. 89, 1989.

6. A. Piazzi, C. G. Lo Bianco, and M. Romano, "η³-Splines for the Smooth Path Generation of Wheeled Mobile Robots," *IEEE Trans. Robotics*, vol. 23, 2007. *(primary algorithm reference)*

7. FLIR, "Bumblebee®2 FireWire," 2019. https://www.flir.com.au/products/bumblebee2-firewire/

8. Velodyne, "HDL-32E Manual," July 2015. https://velodynelidar.com

9. Clearpath Robotics, "HUSKY A200," 2019. https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

10. A. Sakai et al., "PythonRobotics: A Python code collection of robotics algorithms," 2016. https://github.com/AtsushiSakai/PythonRobotics *(η³ reference implementation)*

11. T. M. Howard and A. Kelly, "Optimal Rough Terrain Trajectory Generation for Wheeled Mobile Robots," *Int. Journal of Robotics Research*, pp. 141–165, 2007.

---

## Acknowledgements

This project received support from those participating in and supporting the Budj Bim indigenous program. Special thanks to the traditional owners of the Gunditjmara land for welcoming this research, and to Professor Ivan Marusic, Dr Martin Tomko, Mr Lawrence Molloy, Vinod Veluguri, Jazor He, Yuan Xui, Evan Joseph, and Tomas for their guidance and support throughout.

---

*University of Melbourne, Melbourne School of Engineering — Capstone Project CP12-IMA-191, 2019*

"""
generate_screenshots.py
-----------------------
Runs all five modules and saves their matplotlib output to screenshots/.
Run once from the repo root to regenerate evidence figures.
"""

import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Ensure packages are importable from repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "path_planning"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sensor_coverage"))

OUT = os.path.join(os.path.dirname(__file__), "screenshots")
os.makedirs(OUT, exist_ok=True)

saved = []

def save_all_figs(prefix):
    for i, fig in enumerate(map(plt.figure, plt.get_fignums()), start=1):
        path = os.path.join(OUT, f"{prefix}_{i}.png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        saved.append(path)
        print(f"  saved: {path}")
    plt.close("all")


# ------------------------------------------------------------------
# 1. Dynamic path planner
# ------------------------------------------------------------------
print("Running dynamic_path_planner...")
import dynamic_path_planner as dpp
_, positions = dpp.plan_loop_path([[0, 0], [1, 0], [1, 1]])
dpp.plot_path([[0, 0], [1, 0], [1, 1]], positions)
save_all_figs("1_dynamic_path_planner")


# ------------------------------------------------------------------
# 2. Trajectory planner
# ------------------------------------------------------------------
print("Running trajectory_planner...")
import trajectory_planner as tp
tp.run_capstone_scenario()
save_all_figs("2_trajectory_planner")


# ------------------------------------------------------------------
# 3. LIDAR coverage
# ------------------------------------------------------------------
print("Running lidar_coverage...")
import lidar_coverage as lc
building, flight_path, standoff = lc.plan_lidar_path(
    [[0, 0], [0, 5], [5, 5], [5, 0]], building_height=5.0
)
lc.plot_lidar_coverage(building, flight_path)
save_all_figs("3_lidar_coverage")


# ------------------------------------------------------------------
# 4. Photogrammetry basic
# ------------------------------------------------------------------
print("Running photogrammetry_basic...")
import photogrammetry_basic as pb
positions_b = pb.compute_camera_positions([[0,0],[1,0],[1,1],[0,1]], 5.0)
print("  camera positions:", positions_b)
fig, ax = plt.subplots()
pb.draw_building([[0,0],[1,0],[1,1],[0,1]])
ax.plot([p[0] for p in positions_b], [p[1] for p in positions_b],
        "ro", markersize=8, label="Camera positions")
ax.set_xlabel("m"); ax.set_ylabel("m")
ax.set_title("Photogrammetry Planner — Simple Building")
ax.legend(); ax.set_aspect("equal")
plt.tight_layout()
save_all_figs("4_photogrammetry_basic")


# ------------------------------------------------------------------
# 5. Photogrammetry advanced
# ------------------------------------------------------------------
print("Running photogrammetry_advanced...")
import photogrammetry_advanced as pa
L_SHAPE = [[0,0],[10,0],[10,5],[5,5],[5,10],[0,10]]
positions_a = pa.compute_camera_positions(L_SHAPE, 5.0)
print("  camera positions:", positions_a)
fig, ax = plt.subplots()
pa.draw_building(L_SHAPE)
ax.plot([p[0] for p in positions_a], [p[1] for p in positions_a],
        "ro", markersize=8, label="Camera positions")
ax.set_xlabel("m"); ax.set_ylabel("m")
ax.set_title("Photogrammetry Planner — L-shaped Building")
ax.legend(); ax.set_aspect("equal")
plt.tight_layout()
save_all_figs("5_photogrammetry_advanced")


# ------------------------------------------------------------------
print(f"\nDone — {len(saved)} screenshots saved to screenshots/")

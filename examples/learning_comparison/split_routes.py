"""Predeclare the flight split and manoeuvre parameters before collecting observations."""

import json
from pathlib import Path
from iet_ch5.trajectories import load_trajectory

ROOT = Path(__file__).parent / "routes"
TRAIN = ("wide_turn", "reverse_turn", "climb_turn", "descending", "fast_eight", "yaw_eight")
VALID = "validation_turn"
TEST = "heldout_turn"
if __name__ == "__main__":
    settings = [
        (TRAIN[0], 3.4, 1.6, 0.6, 0.10, 0.12, 8.7, 0.60),
        (TRAIN[1], -2.8, -1.75, 0.8, 0.24, -0.15, 8.5, -0.62),
        (TRAIN[2], 2.9, -1.3, 1.4, 0.25, 0.24, 9.0, 0.67),
        (TRAIN[3], -3.2, 1.5, -1.1, -0.18, 0.2, 8.8, -0.58),
        (TRAIN[4], 3.2, 1.8, -0.6, -0.22, 0.13, 8.4, -0.65),
        (TRAIN[5], -3.0, -1.7, 0.7, -0.2, -0.18, 8.7, 0.68),
        (VALID, 2.7, -1.45, 0.95, -0.12, 0.17, 9.1, -0.52),
        (TEST, -3.1, 1.55, 0.85, 0.13, -0.16, 8.9, 0.54),
    ]
    for name, x, y, z, y3, z3, duration, yaw in settings:
        doc = load_trajectory("agile_course")
        doc.update(id=name, description="Coupled translation and varying heading: " + name)
        doc["timing"]["duration_s"] = duration
        doc["shape"] = dict(zip(doc["shape"], (x, y, z, y3, z3)))
        doc["heading"]["oscillation_rad"] = yaw
        (ROOT / (name + ".json")).write_text(json.dumps(doc, indent=2) + "\n")

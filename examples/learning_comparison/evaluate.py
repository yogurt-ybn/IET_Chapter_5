"""Evaluate frozen controllers on held-out AirSim flights with the common runner."""

import argparse
import json
from pathlib import Path
from excitation import flight
from controllers import IdentifiedMPC, DirectPolicy

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("environment", "models", "output"):
        p.add_argument(key, type=Path)
    args = p.parse_args()
    env = json.loads(args.environment.read_text())
    for trajectory in ("figure_eight", "agile_course"):
        for repeat in (0, 1):
            for name, controller, key in [
                ("dmdc_mpc", IdentifiedMPC, "model"),
                ("ars_policy", DirectPolicy, "policy"),
            ]:
                config = {key: str(args.models / (key + ".npz"))}
                result = flight(
                    name,
                    controller,
                    config,
                    trajectory,
                    args.output / f"{trajectory}-{name}-r{repeat}",
                    env,
                )
                print(
                    result.run_id,
                    result.status,
                    result.metrics.get("loop_rms_m"),
                    result.failure,
                    flush=True,
                )

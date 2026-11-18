"""Compare frozen methods on an existing diagnostic and a new held-out route."""

import sys, json, hashlib
from pathlib import Path
from excitation import flight
from controllers import IdentifiedMPC, DirectPolicy
from wide_control import WideMPC, WidePolicy
from split_routes import ROOT, TEST
from iet_ch5.controllers.geometric.controller import Geometric
from iet_ch5.controllers.configuration import load_controller_configs

if __name__ == "__main__":
    env = json.loads(Path(sys.argv[1]).read_text())
    out, raw = map(Path, sys.argv[2:4])
    old = Path("results/examples/learning_comparison")
    variants = [
        ("old_mpc", IdentifiedMPC, {"model": str(old / "model.npz")}),
        ("old_ars", DirectPolicy, {"policy": str(old / "policy.npz")}),
        ("wide_linear_same_mpc", IdentifiedMPC, {"model": str(out / "linear.npz")}),
        ("wide_linear_preview", WideMPC, {"model": str(out / "linear.npz")}),
        ("wide_nonlinear", WideMPC, {"model": str(out / "nonlinear.npz")}),
        ("policy_initial", WidePolicy, {"policy": str(out / "initial.npz")}),
        *[(f"ars_{s}", WidePolicy, {"policy": str(out / f"ars-{s}.npz")}) for s in (731, 732, 733)],
        ("geometric", Geometric, load_controller_configs()["geometric"]),
    ]
    frozen = {
        f.name: hashlib.sha256(f.read_bytes()).hexdigest()
        for f in out.glob("*.npz")
        if not f.name.startswith("fit-")
    }
    (out / "frozen.json").write_text(json.dumps(frozen, indent=2) + "\n")
    for key, route in [("agile_course", "agile_course"), (TEST, str(ROOT / (TEST + ".json")))]:
        for name, controller, cfg in variants:
            res = flight(name, controller, cfg, route, raw / f"{key}-{name}", env)
            print(key, name, res.status, res.metrics.get("loop_rms_m"), res.failure, flush=True)

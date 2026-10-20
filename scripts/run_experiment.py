#!/usr/bin/env python3
"""Run one controller through the common motor-level AirSim bridge."""

import argparse
import json
from pathlib import Path

from iet_ch5.logging import dump
from iet_ch5.runner import run


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local-config", type=Path, required=True)
    parser.add_argument("--exclusive-simulator", action="store_true", required=True)
    parser.add_argument("--controller", choices=["pid", "lqr", "geometric"], required=True)
    parser.add_argument("--task", default="hover_transfer", choices=["hover_transfer", "circle"])
    parser.add_argument("--controllers", type=Path, default=Path("configs/controllers.json"))
    parser.add_argument("--tasks", type=Path, default=Path("configs/tasks.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    result = run(json.loads(args.local_config.read_text()), json.loads(args.controllers.read_text()),
                 json.loads(args.tasks.read_text()), args.controller, args.task, args.output, seed=args.seed)
    print(dump(result))
    raise SystemExit(0 if result.status == "passed" else 1)

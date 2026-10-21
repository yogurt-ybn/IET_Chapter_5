#!/usr/bin/env python3
"""Unified Chapter 5 controller / trajectory / experiment / media entry point."""

import argparse
import json
from pathlib import Path

from iet_ch5.controllers.configuration import METHODS
from iet_ch5.experiments.configuration import resolve
from iet_ch5.experiments.engine import atomic_json, execute
from iet_ch5.trajectories import available_trajectories


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['list', 'plan', 'run', 'demo', 'publish'])
    parser.add_argument('--suite', default='configs/experiments/case_study.json')
    parser.add_argument('--runtime', default='configs/experiments/runtime.json')
    parser.add_argument('--controller', nargs='+')
    parser.add_argument('--trajectory', nargs='+')
    parser.add_argument('--repeats', type=int)
    parser.add_argument('--controller-overrides')
    parser.add_argument('--local-config', type=Path)
    parser.add_argument('--exclusive-simulator', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--destination', type=Path)
    args = parser.parse_args()
    if args.command == 'list':
        print('Controllers:', ', '.join(METHODS))
        print('Trajectories:', ', '.join(available_trajectories()))
        return
    if not args.output:
        parser.error('--output required')
    if args.command == 'publish':
        if not args.destination:
            parser.error('--destination required')
        from iet_ch5.experiments.publication import publish
        publish(args.output, args.destination)
        return
    if args.command == 'demo' and args.repeats is None:
        args.repeats = 1
    resolved = resolve(args.suite, controllers=args.controller, trajectories=args.trajectory,
                       repeats=args.repeats, controller_overrides=args.controller_overrides,
                       runtime_path=args.runtime)
    if args.command == 'plan':
        args.output.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(args.output, resolved)
        print(f"Frozen plan: {len(resolved['matrix'])} runs")
        return
    if not args.local_config or not args.exclusive_simulator:
        parser.error('run/demo requires --local-config and --exclusive-simulator for an owned simulator')
    execute(resolved, json.loads(args.local_config.read_text()), args.output,
            resume=args.resume, demo=args.command == 'demo')


if __name__ == '__main__':
    main()

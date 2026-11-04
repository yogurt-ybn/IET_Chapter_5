#!/usr/bin/env python3
"""Flight-level GP force-residual learning and matched rigid-body NMPC case study."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['collect','dataset','fit','validate','compare','publish','plot','prediction-plot'])
    p.add_argument('--source', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--dataset', type=Path)
    p.add_argument('--models', type=Path)
    p.add_argument('--validation', type=Path)
    p.add_argument('--local-config', type=Path)
    p.add_argument('--exclusive-simulator', action='store_true')
    p.add_argument('--suite', type=Path)
    p.add_argument('--support-count', type=int, default=64)
    args = p.parse_args()
    required = {'collect':['local_config'], 'dataset':['source'], 'fit':['dataset'],
                'validate':['dataset','models'], 'compare':['models','validation','local_config'],
                'publish':['source','dataset','models','validation'], 'plot':['source'], 'prediction-plot':['source']}
    for name in required[args.command]:
        if getattr(args,name) is None:
            p.error('--'+name.replace('_','-')+' required')
    if args.command in ('collect','compare') and not args.exclusive_simulator:
        p.error('collect/compare requires --exclusive-simulator for an owned simulator')
    if args.command == 'collect':
        from iet_ch5.experiments.configuration import resolve
        from iet_ch5.experiments.engine import execute
        execute(resolve(args.suite or 'configs/learning/collection.json'),
                json.loads(args.local_config.read_text()),args.output)
    elif args.command == 'dataset':
        from iet_ch5.learning.dataset import build_dataset
        build_dataset(args.source,args.output,['transfer','circle','helix'],['figure_eight','spatial_course'])
    elif args.command == 'fit':
        from iet_ch5.learning.workflow import fit_models
        fit_models(args.dataset,args.output,args.support_count)
    elif args.command == 'validate':
        from iet_ch5.learning.workflow import validate_models
        validate_models(args.dataset,args.models,args.output)
    elif args.command == 'compare':
        from iet_ch5.learning.workflow import compare
        compare(args.models,args.validation,args.output,json.loads(args.local_config.read_text()),
                args.suite or 'configs/learning/evaluation.json',exclusive=args.exclusive_simulator)
    elif args.command == 'publish':
        from iet_ch5.learning.publication import publish
        publish(args.source,args.dataset,args.models,args.validation,args.output)
    elif args.command == 'prediction-plot':
        from iet_ch5.learning.publication import plot_prediction
        plot_prediction(args.source,args.output)
    else:
        from iet_ch5.learning.publication import plot
        plot(args.source,args.output)


if __name__ == '__main__':
    main()

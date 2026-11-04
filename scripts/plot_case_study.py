#!/usr/bin/env python3
"""Regenerate publication plots from committed compact experiment data."""
import argparse
import json
from pathlib import Path
import numpy as np
from iet_ch5.experiments.publication import plot

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    rows = json.loads((a.source/'summary.json').read_text())
    data = {r['run_id']: dict(np.load(a.source/'runs'/r['run_id']/'metrics_samples.npz')) for r in rows}
    plot(rows, data, a.output)

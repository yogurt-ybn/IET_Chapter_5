"""Versioned line-oriented common run log and exact configuration provenance."""

import hashlib
import json
import subprocess
from dataclasses import asdict, is_dataclass

import numpy as np


def encode(value):
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def dump(value):
    return json.dumps(value, default=encode, separators=(",", ":"), allow_nan=False)


def config_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def revision():
    return {"commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
            "dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], text=True)),
            "tracked_diff_sha256": hashlib.sha256(subprocess.check_output(
                ["git", "diff", "HEAD", "--", "src", "scripts", "configs"])).hexdigest()}

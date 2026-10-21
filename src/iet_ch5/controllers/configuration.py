"""Controller-owned defaults with strict deep overrides; no implicit simulator settings."""

from copy import deepcopy
from importlib.resources import files
import json
import math
from pathlib import Path

METHODS = ('pid', 'lqr', 'geometric', 'dob', 'mpc', 'nmpc')


def merge_parameters(base, override, prefix=''):
    if not isinstance(override, dict):
        raise ValueError("Controller overrides must be JSON objects")
    result = deepcopy(base)
    for key, value in override.items():
        name = f'{prefix}.{key}' if prefix else key
        if key not in base:
            raise ValueError(f'Unknown configuration key: {name}')
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f'Expected an object for {name}')
            result[key] = merge_parameters(base[key], value, name)
        else:
            if isinstance(base[key], list) and (not isinstance(value, list) or len(value) != len(base[key])):
                raise ValueError(f'Wrong array size for {name}')
            result[key] = deepcopy(value)
    return result


def load_controller_configs(path=None, overrides=None):
    """One JSON per controller; optional legacy full map and strict partial overrides."""
    defaults = {name: json.loads(files(f'iet_ch5.controllers.{name}').joinpath('config.json').read_text())
                for name in METHODS}
    if path is not None:
        defaults = merge_parameters(defaults, json.loads(Path(path).read_text()))
    if overrides:
        defaults = merge_parameters(defaults, overrides)
    def validate(value, key=""):
        if isinstance(value, dict):
            for name, item in value.items():
                validate(item, name)
        elif isinstance(value, list):
            for item in value:
                validate(item, key)
        elif isinstance(value, bool):
            if key not in ("enabled", "warm_start"):
                raise ValueError(f"Expected numeric parameter: {key}")
        elif isinstance(value, str):
            if key != 'solver_type' or value not in ('SQP_RTI', 'SQP'):
                raise ValueError(f'Invalid solver enumeration: {key}={value}')
        elif not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"Expected positive finite parameter: {key}")
        elif key in ("horizon", "max_iter", "qp_max_iter", "integration_steps", "condensing_stages") and not isinstance(value, int):
            raise ValueError(f"Expected integer: {key}")
    validate(defaults)
    if defaults['nmpc']['condensing_stages'] > defaults['nmpc']['horizon']:
        raise ValueError('condensing_stages must not exceed horizon')
    return defaults

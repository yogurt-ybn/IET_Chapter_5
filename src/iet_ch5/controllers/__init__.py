"""Numerical controllers. Every implementation returns physical collective thrust/moments."""


def make_controller(name, config=None):
    import importlib
    if config is None:
        from .configuration import load_controller_configs
        config = load_controller_configs()[name]
    classes = {"pid": "PID", "lqr": "LQR", "geometric": "Geometric", "dob": "DOB", "mpc": "MPC", "nmpc": "NMPC"}
    module = importlib.import_module(f"{__name__}.{name}")
    return getattr(module, classes[name])(config)

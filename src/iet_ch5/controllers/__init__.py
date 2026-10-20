"""Numerical controllers. Every implementation returns physical collective thrust/moments."""


def make_controller(name, config):
    import importlib
    classes = {"pid": "PID", "lqr": "LQR", "geometric": "Geometric", "dob": "DOB", "mpc": "MPC"}
    module = importlib.import_module(f"{__name__}.{name}")
    return getattr(module, classes[name])(config)

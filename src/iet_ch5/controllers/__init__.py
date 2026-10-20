"""Numerical controllers. Every implementation returns physical collective thrust/moments."""


def make_controller(name, config):
    from .pid import PID
    from .lqr import LQR
    from .geometric import Geometric
    classes = {"pid": PID, "lqr": LQR, "geometric": Geometric}
    return classes[name](config)

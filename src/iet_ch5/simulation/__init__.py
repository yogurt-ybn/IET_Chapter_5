"""Backend selection and simulation contract; one shared physical control chain."""

from .interface import SimulationBackend, SceneImage


def create_backend(config, *, exclusive=False):
    if config.get('backend', 'airsim') != 'airsim':
        raise ValueError('Only the measured AirSim backend is implemented')
    from .airsim_backend import AirSimBackend
    return AirSimBackend(config['host'], config['port'], config['vehicle'], exclusive=exclusive)


__all__ = ['SimulationBackend', 'SceneImage', 'create_backend']

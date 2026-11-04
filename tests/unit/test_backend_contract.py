"""Runner contract test with an explicit oracle backend, not simulated-flight evidence."""

from pathlib import Path

import numpy as np
import pytest

from iet_ch5.allocation import Allocator
from iet_ch5.controllers.configuration import load_controller_configs
from iet_ch5.experiments.configuration import build_runtime
from iet_ch5.runner import run
from iet_ch5.simulation import SimulationBackend
from iet_ch5.trajectories import load_trajectory
from iet_ch5.types import State, VehicleParameters


class HoverOracle:
    """Fixed truth isolates wiring/clock/cleanup, without claiming plant dynamics."""
    def __init__(self, fail_step=False):
        self.parameters = VehicleParameters(1., np.diag([.007, .008, .014]), 9.81,
            np.array([[.16,-.16,0],[-.16,.16,0],[.16,.16,0],[-.16,-.16,0]]),
            np.tile([0.,0.,1.],(4,1)), np.array([-1,-1,1,1]),4.2,.056,.99,.005,.003)
        self.last_timing = {}
        self.provenance = {'kind': 'unit-test fixed-state oracle, not AirSim'}
        self.time = 1_000_000_000
        self.steps = []
        self.stopped = self.closed = False
        self.fail_step = fail_step

    def initialize(self, position):
        self.position = np.asarray(position)

    def set_initial_kinematics(self, position):
        self.position = np.asarray(position)

    def paused_truth(self, timestamp_ns):
        return State(self.position.copy(),np.zeros(3),np.eye(3),np.zeros(3),timestamp_ns,'oracle')

    def step(self, pwm, duration):
        if self.fail_step:
            raise RuntimeError('injected backend failure')
        self.steps.append(np.asarray(pwm))
        self.time += 15_000_000 if len(self.steps)%2 else 12_000_000
        sample = {'timestamp':self.time,'rotors':[{'control_signal_filtered':float(p)} for p in pwm],
                  'has_collided':False}
        return self.paused_truth(self.time), [sample], .001

    def decode_physics_sample(self, sample):
        return self.paused_truth(sample['timestamp'])

    def set_wind(self, velocity_nwu):
        raise NotImplementedError

    def configure_camera(self, *args):
        raise NotImplementedError

    def camera_info(self, *args, **kwargs):
        raise NotImplementedError

    def draw_paths(self, *args):
        raise NotImplementedError

    def capture_scene(self, *args, **kwargs):
        raise NotImplementedError

    def stop(self):
        self.stopped = True

    def close(self):
        self.closed = True


@pytest.mark.parametrize('fail', [False, True])
def test_injected_backend_clock_allocation_and_cleanup(tmp_path, fail, monkeypatch):
    import iet_ch5.runner as runner
    monkeypatch.setattr(runner, 'create_backend', lambda *a, **kw: pytest.fail('Unexpected AirSim dependency'))
    backend = HoverOracle(fail)
    assert isinstance(backend, SimulationBackend)
    trajectory = load_trajectory('hover')
    trajectory['timing'].update(onset_s=.012,duration_s=.048,final_hold_s=.012)
    common = {'control_period_s':.012,'settling_tolerance_m':.1,'settling_dwell_s':.012,'feedback':{}}
    runtime = build_runtime(trajectory, {'kind':'nominal'}, common)
    result = run({}, load_controller_configs(), runtime, 'pid', 'hover', tmp_path/'run', backend=backend)
    assert backend.stopped and backend.closed
    if fail:
        assert result.status == 'failed' and 'injected backend failure' in result.failure
        assert (tmp_path/'run'/'failure.txt').exists()
    else:
        assert result.status == 'passed'
        expected = Allocator(backend.parameters).inverse@np.array([9.81,0,0,0])
        assert np.allclose(backend.steps,expected)
        assert result.metrics['position_rms_m'] == 0
        assert Path(result.log_path).exists()

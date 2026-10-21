"""Fixed external-camera evidence, sampled outside the control timing interval."""

import hashlib

import numpy as np

from .frames import C, vector, rotation
from .logging import dump


def ned_points(points, origin):
    return [dict(zip(['x_val', 'y_val', 'z_val'], (origin + C @ p).tolist())) for p in points]


class SceneCapture:
    def __init__(self, camera='tracking_overview', period=.1, position=None):
        self.config = {'camera': camera, 'external': True, 'period_s': period,
                       'camera_position_vehicle_origin_ned_m': (
                           [2., -4., -6.] if position is None else list(position)),
                       'reference_color': 'yellow', 'actual_trace_color': 'cyan',
                       'timing': 'capture excluded from controller/loop timing'}
        self.frames = []
        self.trace = []

    def initialize(self, backend, task, duration, state, output):
        self.backend, self.task, self.output = backend, task, output
        (output / 'frames').mkdir()
        self.reference = np.array([task.at(t).p for t in np.linspace(0, duration, 401)])
        # AirSim kinematics are relative to the vehicle's initial origin; native debug
        # markers and external cameras use GLOBAL NED. The spawn/ground adjustment
        # makes this translation different from the configured Vehicles.Z value.
        target = (self.reference.max(axis=0)+self.reference.min(axis=0))/2
        setup = backend.configure_camera(self.config['camera'],
                    C@np.array(self.config['camera_position_vehicle_origin_ned_m']), target)
        self.camera_info = setup['camera_info']
        self.origin = setup['measured_vehicle_origin_global_ned_m']
        self.next_capture = 0.
        self.update(state, state.timestamp_ns)

    def update(self, state, initial_timestamp):
        t = (state.timestamp_ns - initial_timestamp) * 1e-9
        if t + 1e-9 < self.next_capture:
            return
        camera = self.backend.camera_info(self.config['camera'])
        # A fixed camera is a checked invariant, not just a requested view mode.
        original = self.camera_info['pose']
        current = camera['pose']
        if (not np.allclose(vector(original['position']), vector(current['position']), atol=1e-5)
                or not np.allclose(rotation(original['orientation']),
                                   rotation(current['orientation']), atol=1e-5)):
            raise RuntimeError('External evidence camera moved during the run')
        self.trace.append(state.p.copy())
        self.backend.draw_paths(self.reference, self.trace)
        image = self.backend.capture_scene(self.config['camera'], external=True)
        raw = image.png
        filename = f'{len(self.frames):05d}.png'
        (self.output / 'frames' / filename).write_bytes(raw)
        self.frames.append({'filename': filename, 'simulation_t': t,
                            'truth_timestamp_ns': state.timestamp_ns,
                            'image_timestamp_ns': image.timestamp_ns,
                            'truth_p_nwu_m': state.p, 'reference_p_nwu_m': self.task.at(t).p,
                            'truth_v_nwu_m_s': state.v, 'reference_v_nwu_m_s': self.task.at(t).v,
                            'camera_info': camera, 'sha256': hashlib.sha256(raw).hexdigest()})
        (self.output / 'frames.json').write_text(dump(self.frames) + '\n')
        (self.output / 'camera_provenance.json').write_text(dump({
            **self.config, 'camera_info': self.camera_info,
            'evidence': 'Actual AirSim scene frames with native debug reference/trace markers',
            'timestamps': 'Physics truth timestamp at paused image acquisition; no assumed frame dt',
            'coordinate_frame': 'logged paths vehicle-origin NWU; markers global NED',
            'measured_vehicle_origin_global_ned_m': self.origin,
            'origin_measurement': 'simGetObjectPose(vehicle) minus simGetVehiclePose(vehicle)',
        }) + '\n')
        self.next_capture = (np.floor((t + 1e-9) / self.config['period_s']) + 1) * self.config['period_s']

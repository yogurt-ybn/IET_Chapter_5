"""Fixed external-camera evidence, sampled outside the control timing interval."""

import hashlib

import numpy as np

from .frames import C, vector, rotation
from .logging import dump


def ned_points(points):
    return [dict(zip(['x_val', 'y_val', 'z_val'], (C @ p).tolist())) for p in points]


class SceneCapture:
    def __init__(self, camera='tracking_overview', period=.1):
        self.config = {'camera': camera, 'external': True, 'period_s': period,
                       'reference_color': 'yellow', 'actual_trace_color': 'cyan',
                       'timing': 'capture excluded from controller/loop timing'}
        self.frames = []
        self.trace = []

    def initialize(self, backend, task, duration, state, output):
        self.backend, self.task, self.output = backend, task, output
        (output / 'frames').mkdir()
        self.reference = np.array([task.at(t).p for t in np.linspace(0, duration, 401)])
        self.camera_info = backend.rpc.call('simGetCameraInfo', self.config['camera'], '', True)
        self.next_capture = 0.
        self.update(state, state.timestamp_ns)

    def update(self, state, initial_timestamp):
        t = (state.timestamp_ns - initial_timestamp) * 1e-9
        if t + 1e-9 < self.next_capture:
            return
        rpc = self.backend.rpc
        camera = rpc.call('simGetCameraInfo', self.config['camera'], '', True)
        # A fixed camera is a checked invariant, not just a requested view mode.
        original = self.camera_info['pose']
        current = camera['pose']
        if (not np.allclose(vector(original['position']), vector(current['position']), atol=1e-5)
                or not np.allclose(rotation(original['orientation']),
                                   rotation(current['orientation']), atol=1e-5)):
            raise RuntimeError('External evidence camera moved during the run')
        self.trace.append(state.p.copy())
        rpc.call('simFlushPersistentMarkers')
        rpc.call('simPlotLineStrip', ned_points(self.reference), [1., .8, 0., 1.], 5., -1., True)
        if len(self.trace) > 1:
            rpc.call('simPlotLineStrip', ned_points(self.trace), [0., 1., 1., 1.], 3., -1., True)
        request = {'camera_name': self.config['camera'], 'image_type': 0,
                   'pixels_as_float': False, 'compress': True}
        image = rpc.call('simGetImages', [request], '', True)[0]
        raw = image['image_data_uint8']
        raw = raw.encode('latin1') if isinstance(raw, str) else bytes(raw)
        if image['width'] <= 0 or image['height'] <= 0 or not raw:
            raise RuntimeError('External scene capture returned an empty frame')
        filename = f'{len(self.frames):05d}.png'
        (self.output / 'frames' / filename).write_bytes(raw)
        self.frames.append({'filename': filename, 'simulation_t': t,
                            'truth_timestamp_ns': state.timestamp_ns,
                            'image_timestamp_ns': image['time_stamp'],
                            'truth_p_nwu_m': state.p, 'reference_p_nwu_m': self.task.at(t).p,
                            'camera_info': camera, 'sha256': hashlib.sha256(raw).hexdigest()})
        (self.output / 'frames.json').write_text(dump(self.frames) + '\n')
        (self.output / 'camera_provenance.json').write_text(dump({
            **self.config, 'camera_info': self.camera_info,
            'evidence': 'Actual AirSim scene frames with native debug reference/trace markers',
            'timestamps': 'Physics truth timestamp at paused image acquisition; no assumed frame dt',
            'coordinate_frame': 'logged paths NWU; simulator markers converted to world NED',
        }) + '\n')
        self.next_capture = (np.floor((t + 1e-9) / self.config['period_s']) + 1) * self.config['period_s']

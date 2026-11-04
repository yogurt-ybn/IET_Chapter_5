"""Motor-level AirSim bridge; all actions require a designated isolated server.

Control uses simulator time. It never substitutes AirSim's built-in flight controller.
The two RPC sockets keep long motor calls from blocking time advancement.
"""

import time
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..backends.rpc import RpcClient
from ..frames import parameters_from_rpc, state_from_sample
from ..frames import C, vector
from .interface import SceneImage


class AirSimBackend:
    def __init__(self, host, port, vehicle, *, exclusive=False):
        self.rpc = RpcClient(host, port)
        self.motor = RpcClient(host, port)
        self.vehicle = vehicle
        self.exclusive = exclusive
        self.parameters = parameters_from_rpc(self.rpc.call("getDynamicsParameters", vehicle))
        self.last_sequence = 0
        self.last_timestamp = None
        self.pending_motors = set()
        inventory = Path('docs/airsim_source_inventory.json')
        self.provenance = json.loads(inventory.read_text()) if inventory.exists() else {
            'backend': 'AirSim', 'source_inventory': 'unavailable in this installation'}

    decode_physics_sample = staticmethod(state_from_sample)

    def camera_info(self, camera, external=True):
        return self.rpc.call('simGetCameraInfo', camera, '' if external else self.vehicle, external)

    def configure_camera(self, camera, position_nwu, target_nwu):
        self._require_exclusive()
        actor = self.rpc.call('simGetObjectPose', self.vehicle)
        vehicle = self.rpc.call('simGetVehiclePose', self.vehicle)
        self.camera_origin_ned = vector(actor['position']) - vector(vehicle['position'])
        position = C @ np.asarray(position_nwu)
        direction = C @ np.asarray(target_nwu) - position
        yaw = np.arctan2(direction[1], direction[0])
        pitch = -np.arctan2(direction[2], np.linalg.norm(direction[:2]))
        q = Rotation.from_euler('xyz', [0., pitch, yaw]).as_quat()
        pose = {'position': dict(zip(['x_val', 'y_val', 'z_val'],
                                    (position+self.camera_origin_ned).tolist())),
                'orientation': dict(zip(['x_val', 'y_val', 'z_val', 'w_val'], q.tolist()))}
        self.rpc.call('simSetCameraPose', camera, pose, '', True)
        return {'camera_info': self.camera_info(camera),
                'measured_vehicle_origin_global_ned_m': self.camera_origin_ned.tolist()}

    def draw_paths(self, reference_nwu, actual_nwu):
        self._require_exclusive()
        def points(values):
            return [dict(zip(['x_val', 'y_val', 'z_val'],
                             (self.camera_origin_ned+C@np.asarray(p)).tolist())) for p in values]
        self.rpc.call('simFlushPersistentMarkers')
        self.rpc.call('simPlotLineStrip', points(reference_nwu), [1., .8, 0., 1.], 5., -1., True)
        if len(actual_nwu) > 1:
            self.rpc.call('simPlotLineStrip', points(actual_nwu), [0., 1., 1., 1.], 3., -1., True)

    def capture_scene(self, camera='0', external=False):
        request = {'camera_name': camera, 'image_type': 0, 'pixels_as_float': False, 'compress': True}
        value = self.rpc.call('simGetImages', [request], '' if external else self.vehicle, external)[0]
        data = value['image_data_uint8']
        data = data.encode('latin1') if isinstance(data, str) else bytes(data)
        if not data or value['width'] <= 0 or value['height'] <= 0:
            raise RuntimeError('Simulator returned an empty scene image')
        return SceneImage(data, int(value['time_stamp']), value['width'], value['height'])

    def _require_exclusive(self):
        if not self.exclusive:
            raise RuntimeError("Vehicle actions require an explicitly designated exclusive simulator")

    def latest(self, *, allow_empty=False):
        batch = self.rpc.call("getDynamicsStateBatch", self.last_sequence, 4096, self.vehicle)
        if batch["samples_dropped"]:
            raise RuntimeError("Physics samples dropped from server ring buffer")
        samples = batch["samples"]
        if not samples:
            if allow_empty:
                return None, []
            raise RuntimeError("No new physics samples; duplicate/stale feedback rejected")
        sequences = np.array([s["sequence"] for s in samples], dtype=np.int64)
        timestamps = np.array([s["timestamp"] for s in samples], dtype=np.int64)
        if np.any(np.diff(sequences) != 1) or np.any(np.diff(timestamps) <= 0):
            raise RuntimeError("Out-of-order, skipped or duplicate physics samples")
        if self.last_sequence and sequences[0] != self.last_sequence + 1:
            raise RuntimeError("Noncontiguous physics sequence")
        self.last_sequence = int(sequences[-1])
        self.last_timestamp = int(timestamps[-1])
        return state_from_sample(samples[-1]), samples

    def initialize(self, position=(0., 0., 2.)):
        self._require_exclusive()
        self.rpc.call("simPause", True)
        self.rpc.call("reset")
        self.set_wind(np.zeros(3))
        self.rpc.call("enableApiControl", True, self.vehicle)
        if not self.rpc.call("armDisarm", True, self.vehicle):
            raise RuntimeError("Vehicle refused to arm")
        self.set_initial_kinematics(position)
        self.last_sequence = 0
        self.last_timestamp = None

    def set_initial_kinematics(self, position):
        self._require_exclusive()
        # A clearly separated setup operation. No teleportation occurs during evaluated intervals.
        x, y, z = position
        k = {"position": {"x_val": x, "y_val": -y, "z_val": -z},
             "orientation": {"w_val": 1., "x_val": 0., "y_val": 0., "z_val": 0.},
             "linear_velocity": {"x_val": 0., "y_val": 0., "z_val": 0.},
             "angular_velocity": {"x_val": 0., "y_val": 0., "z_val": 0.},
             "linear_acceleration": {"x_val": 0., "y_val": 0., "z_val": 0.},
             "angular_acceleration": {"x_val": 0., "y_val": 0., "z_val": 0.}}
        self.rpc.call("simSetKinematics", k, True, self.vehicle)

    def paused_truth(self, timestamp_ns):
        if not self.rpc.call("simIsPaused"):
            raise RuntimeError("Timestamped direct truth read requires a paused simulator")
        state = state_from_sample({"kinematics":self.rpc.call("simGetGroundTruthKinematics",self.vehicle),
                                   "timestamp":timestamp_ns})
        state.provenance = "AirSim paused direct ground truth at last physics timestamp"
        return state

    def set_wind(self, velocity_nwu):
        self._require_exclusive()
        v = C @ np.asarray(velocity_nwu, dtype=float)
        self.rpc.call("simSetWind", dict(zip(["x_val", "y_val", "z_val"], v.tolist())))

    def step(self, pwm, duration):
        self._require_exclusive()
        pwm = np.asarray(pwm, dtype=float)
        if pwm.shape != (4,) or not np.isfinite(pwm).all() or np.any((pwm < 0) | (pwm > 1)):
            raise ValueError("Expected four finite normalized motor inputs in [0,1]")
        start = time.perf_counter()
        request = self.motor.submit("moveByMotorPWMs", *pwm.tolist(), 1e-6, self.vehicle)
        self.pending_motors.add(request)
        # The source executes commandMotorPWMs once before its duration wait. Cancellation
        # ends that wait but preserves the installed passthrough goal (verified in M0).
        # Wait for this acknowledgement BEFORE stepping. Repeated bounded cancellation
        # also handles the worker not having started at the first cancellation request.
        # Unlike joining an uncancelled future, this does not require physics to advance.
        deadline = time.monotonic()+self.rpc.timeout
        while request in self.pending_motors:
            self.rpc.call("cancelLastTask",self.vehicle)
            for response in self.motor.drain():
                self.pending_motors.discard(response[1])
            if time.monotonic()>deadline:
                raise TimeoutError("Cancelled motor request did not acknowledge installed goal")
            if request in self.pending_motors:
                time.sleep(.0002)
        acknowledged = time.perf_counter()
        # ScheduledExecutor::continueForTime measures WALL time, not SteppableClock time.
        # Under scheduling load one call can yield fewer physics ticks than requested.
        # Advance to a measured simulation timestamp, accumulating every intermediate sample.
        initial_stamp = self.last_timestamp
        if initial_stamp is None:
            initial_stamp = self.rpc.call("getMultirotorState",self.vehicle)["timestamp"]
        elapsed = 0.
        samples = []
        continuations = 0
        advancement_rpc_s = 0.
        state_rpc_s = 0.
        while elapsed < duration:
            remaining = max(duration-elapsed,2*self.parameters.physics_dt)
            before_rpc = time.perf_counter()
            self.rpc.call("simContinueForTime",float(remaining))
            advancement_rpc_s += time.perf_counter()-before_rpc
            before_rpc = time.perf_counter()
            new_state, new_samples = self.latest(allow_empty=True)
            state_rpc_s += time.perf_counter()-before_rpc
            continuations += 1
            if new_samples:
                samples.extend(new_samples)
                state = new_state
                elapsed = (state.timestamp_ns-initial_stamp)*1e-9
            if time.monotonic()>deadline:
                raise TimeoutError("Physics did not reach requested simulation timestamp")
        self.last_timing = {"motor_ack_s":acknowledged-start,
                            "physics_advance_s":advancement_rpc_s,
                            "state_rpc_s":state_rpc_s,
                            "continuations":continuations,
                            "requested_simulation_dt_s":duration,
                            "measured_simulation_dt_s":elapsed}
        actual = np.array([r["control_signal_input"] for r in samples[-1]["rotors"]])
        if not np.allclose(actual, pwm, atol=2e-6, rtol=0):
            self.failed_step_samples = samples
            raise RuntimeError(f"Motor goal not applied: requested {pwm}, observed {actual}")
        return state, samples, time.perf_counter() - start

    def stop(self):
        self._require_exclusive()
        self.rpc.call("simPause", True)
        self.set_wind(np.zeros(3))
        self.rpc.call("armDisarm", False, self.vehicle)
        self.rpc.call("enableApiControl", False, self.vehicle)

    def close(self):
        self.rpc.close()
        self.motor.close()

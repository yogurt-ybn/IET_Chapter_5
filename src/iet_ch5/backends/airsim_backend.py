"""Motor-level AirSim bridge; all actions require a designated isolated server.

Control uses simulator time. It never substitutes AirSim's built-in flight controller.
The two RPC sockets keep long motor calls from blocking time advancement.
"""

import time

import numpy as np

from .rpc import RpcClient
from ..frames import parameters_from_rpc, state_from_sample


class AirSimBackend:
    def __init__(self, host, port, vehicle, *, exclusive=False):
        self.rpc = RpcClient(host, port)
        self.motor = RpcClient(host, port)
        self.vehicle = vehicle
        self.exclusive = exclusive
        self.parameters = parameters_from_rpc(self.rpc.call("getDynamicsParameters", vehicle))
        self.last_sequence = 0
        self.pending_motors = set()

    def _require_exclusive(self):
        if not self.exclusive:
            raise RuntimeError("Vehicle actions require an explicitly designated exclusive simulator")

    def latest(self):
        batch = self.rpc.call("getDynamicsStateBatch", self.last_sequence, 4096, self.vehicle)
        if batch["samples_dropped"]:
            raise RuntimeError("Physics samples dropped from server ring buffer")
        samples = batch["samples"]
        if not samples:
            raise RuntimeError("No new physics samples; duplicate/stale feedback rejected")
        sequences = np.array([s["sequence"] for s in samples], dtype=np.int64)
        timestamps = np.array([s["timestamp"] for s in samples], dtype=np.int64)
        if np.any(np.diff(sequences) != 1) or np.any(np.diff(timestamps) <= 0):
            raise RuntimeError("Out-of-order, skipped or duplicate physics samples")
        if self.last_sequence and sequences[0] != self.last_sequence + 1:
            raise RuntimeError("Noncontiguous physics sequence")
        self.last_sequence = int(sequences[-1])
        return state_from_sample(samples[-1]), samples

    def initialize(self, position=(0., 0., 2.)):
        self._require_exclusive()
        self.rpc.call("simPause", True)
        self.rpc.call("reset")
        self.rpc.call("enableApiControl", True, self.vehicle)
        if not self.rpc.call("armDisarm", True, self.vehicle):
            raise RuntimeError("Vehicle refused to arm")
        # A clearly separated setup operation. No teleportation occurs during evaluated intervals.
        x, y, z = position
        k = {"position": {"x_val": x, "y_val": -y, "z_val": -z},
             "orientation": {"w_val": 1., "x_val": 0., "y_val": 0., "z_val": 0.},
             "linear_velocity": {"x_val": 0., "y_val": 0., "z_val": 0.},
             "angular_velocity": {"x_val": 0., "y_val": 0., "z_val": 0.},
             "linear_acceleration": {"x_val": 0., "y_val": 0., "z_val": 0.},
             "angular_acceleration": {"x_val": 0., "y_val": 0., "z_val": 0.}}
        self.rpc.call("simSetKinematics", k, True, self.vehicle)
        self.last_sequence = 0

    def step(self, pwm, duration):
        self._require_exclusive()
        pwm = np.asarray(pwm, dtype=float)
        if pwm.shape != (4,) or not np.isfinite(pwm).all() or np.any((pwm < 0) | (pwm > 1)):
            raise ValueError("Expected four finite normalized motor inputs in [0,1]")
        start = time.perf_counter()
        request = self.motor.submit("moveByMotorPWMs", *pwm.tolist(), 1e-6, self.vehicle)
        self.pending_motors.add(request)
        # Let the RPC worker install the passthrough goal before advancing physics.
        # Measured telemetry below is the check; this scheduling delay alone is not a guarantee.
        time.sleep(.001)
        self.rpc.call("simContinueForTime", float(duration))
        # Its worker sleeps for a command period even when duration is shorter. With a paused
        # simulator, a future may therefore remain pending after this step. Never join it here.
        for response in self.motor.drain():
            self.pending_motors.discard(response[1])
        if len(self.pending_motors) > 20:
            raise RuntimeError("Motor request queue stopped progressing")
        state, samples = self.latest()
        actual = np.array([r["control_signal_input"] for r in samples[-1]["rotors"]])
        if not np.allclose(actual, pwm, atol=2e-6, rtol=0):
            raise RuntimeError(f"Motor goal not applied: requested {pwm}, observed {actual}")
        return state, samples, time.perf_counter() - start

    def stop(self):
        self._require_exclusive()
        self.rpc.call("simPause", True)
        self.rpc.call("armDisarm", False, self.vehicle)
        self.rpc.call("enableApiControl", False, self.vehicle)

    def close(self):
        self.rpc.close()
        self.motor.close()

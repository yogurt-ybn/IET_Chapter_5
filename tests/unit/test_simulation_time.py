from types import SimpleNamespace

import numpy as np

from iet_ch5.backends.airsim_backend import AirSimBackend
from iet_ch5.types import State


def test_control_step_accumulates_wall_continuations_to_simulation_deadline():
    b = object.__new__(AirSimBackend)
    b.exclusive,b.vehicle,b.last_timestamp = True,'test',1_000_000_000
    b.parameters = SimpleNamespace(physics_dt=.003)
    b.pending_motors = set()
    requests=[]
    class Rpc:
        timeout=1
        def call(self,method,*args):
            requests.append((method,args))
    class Motor:
        def submit(self,*args):return 1
        def drain(self):return [[1,1,None,False]]
    b.rpc,b.motor=Rpc(),Motor()
    stamps=iter([1_003_000_000,1_009_000_000,1_015_000_000])
    def latest(**kwargs):
        b.last_timestamp=next(stamps)
        state=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),b.last_timestamp)
        return state,[{'rotors':[{'control_signal_input':.6} for _ in range(4)]}]
    b.latest=latest
    state,samples,wall=b.step(np.full(4,.6),.012)
    assert state.timestamp_ns==1_015_000_000
    assert len(samples)==3
    assert b.last_timing['continuations']==3
    assert b.last_timing['measured_simulation_dt_s']>=.012
    assert len([r for r in requests if r[0]=='simContinueForTime'])==3

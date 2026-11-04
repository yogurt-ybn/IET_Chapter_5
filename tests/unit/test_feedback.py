import numpy as np

from iet_ch5.feedback import FeedbackPipeline
from iet_ch5.types import State


def test_delay_interpolation_noise_seed_and_truth_independence():
    c = {"noise_period":.012,"delay_s":.024,"position_std_m":0.,"velocity_std_m_s":0.,
         "attitude_std_rad":0.,"rate_std_rad_s":0.}
    states = [State(np.array([i*.012,0.,2.]),np.array([1.,0.,0.]),np.eye(3),np.zeros(3),
                    1_000_000_000+i*12_000_000) for i in range(10)]
    pipeline = FeedbackPipeline(c,2.,3,states[0])
    pipeline.add_truth(states[1:])
    s,index = pipeline.sample(states[5])
    assert np.allclose(s.p,states[3].p)
    assert s.timestamp_ns == states[3].timestamp_ns
    assert np.allclose(states[5].p,[.06,0,2])
    c['position_std_m']=.01
    first,second = FeedbackPipeline(c,2.,7,states[0]),FeedbackPipeline(c,2.,7,states[0])
    assert np.array_equal(first.noise,second.noise)
    assert not np.array_equal(first.noise,FeedbackPipeline(c,2.,8,states[0]).noise)

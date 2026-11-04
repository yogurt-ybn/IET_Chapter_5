import json
import os
from pathlib import Path

import pytest

from iet_ch5.backends.rpc import RpcClient
from iet_ch5.frames import parameters_from_rpc


@pytest.mark.integration
def test_read_only_parameters_and_batch():
    path = os.environ.get("IET_CH5_LOCAL_CONFIG")
    if not path:
        pytest.skip("AirSim integration disabled: set IET_CH5_LOCAL_CONFIG to an ignored JSON file")
    c = json.loads(Path(path).read_text())
    with RpcClient(c["host"], c["port"], timeout=3) as rpc:
        assert rpc.call("ping")
        assert c["vehicle"] in rpc.call("listVehicles")
        p = parameters_from_rpc(rpc.call("getDynamicsParameters", c["vehicle"]))
        assert p.mass > 0
        batch = rpc.call("getDynamicsStateBatch", 0, 2, c["vehicle"])
        assert "samples_dropped" in batch
        assert len(batch["samples"]) <= 2

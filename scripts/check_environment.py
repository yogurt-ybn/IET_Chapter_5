#!/usr/bin/env python3
"""Read-only RPC capability inspection. Does not arm, reset, pause or command a vehicle."""

import argparse
import hashlib
import json
import platform
import time
from pathlib import Path

from iet_ch5.backends.rpc import RpcClient


def inspect(host, port, vehicle):
    result = {"platform": platform.system(), "python": platform.python_version(),
              "vehicle": vehicle, "capabilities": {}}
    with RpcClient(host, port) as rpc:
        methods = [("ping", ()), ("getServerVersion", ()),
                   ("getMinRequiredClientVersion", ()), ("listVehicles", ()),
                   ("simIsPaused", ()), ("isApiControlEnabled", (vehicle,)),
                   ("getMultirotorState", (vehicle,)), ("getDynamicsParameters", (vehicle,)),
                   ("getRotorStates", (vehicle,)), ("simGetGroundTruthKinematics", (vehicle,)),
                   ("simGetGroundTruthEnvironment", (vehicle,)),
                   ("getDynamicsStateBatch", (0, 4, vehicle))]
        for method, args in methods:
            start = time.perf_counter()
            try:
                value = rpc.call(method, *args)
                result["capabilities"][method] = {"available": True, "result": value,
                                                    "wall_s": time.perf_counter() - start}
            except Exception as error:
                result["capabilities"][method] = {"available": False,
                                                    "error": f"{type(error).__name__}: {error}"}
        settings = rpc.call("getSettingsString")
        result["settings_sha256"] = hashlib.sha256(settings.encode()).hexdigest()
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--local-config", type=Path, required=True,
                   help="Ignored JSON containing host, port and vehicle")
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    c = json.loads(args.local_config.read_text())
    result = inspect(c["host"], c["port"], c["vehicle"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v["available"] for k, v in result["capabilities"].items()}))


if __name__ == "__main__":
    main()

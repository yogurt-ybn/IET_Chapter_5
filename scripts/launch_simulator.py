#!/usr/bin/env python3
"""Launch the installed Blocks scene on a designated private RPC port, without rebuilding."""

import argparse
import json
import os
import socket
import subprocess
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--local-config", type=Path, required=True)
    p.add_argument("--settings", type=Path, default=Path("configs/airsim-settings.json"))
    p.add_argument("--local-output", type=Path, default=Path(".local"))
    args = p.parse_args()
    c = json.loads(args.local_config.read_text())
    settings = json.loads(args.settings.read_text())
    if c["port"] != settings["ApiServerPort"] or c["vehicle"] not in settings["Vehicles"]:
        raise ValueError("Local connection and simulator settings must identify the same vehicle/port")
    try:
        connection = socket.create_connection((c["host"], c["port"]), timeout=.2)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError("Selected port is already occupied; existing process left untouched")
    out = args.local_output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    settings["LocalHostIp"] = c["host"]
    settings_path = out / "settings.json"
    settings_path.write_text(json.dumps(settings, indent=2) + "\n")
    command = [c["editor"], c["project"], "-game", "-RenderOffscreen", "-Unattended",
               "-NoSound", "-NoSplash", "-NoLoadingScreen", f"-settings={settings_path}",
               "-log", f"-abslog={out / 'unreal.log'}", "-ResX=640", "-ResY=480", "-windowed"]
    env = {k: v for k, v in os.environ.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY")}
    with (out / "launch.stdout").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=env)
    (out / "simulator.pid").write_text(str(process.pid) + "\n")
    print(f"Launched isolated simulator pid={process.pid}; read-only capability check is next")


if __name__ == "__main__":
    main()

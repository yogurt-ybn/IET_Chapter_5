"""Capture a separate actual AirSim demo; never include image overhead in comparison timing."""

import argparse
import hashlib
import json
from pathlib import Path
import imageio.v2 as imageio
import numpy as np
from scipy.spatial.transform import Rotation
from iet_ch5.backends.rpc import RpcClient
from iet_ch5.runner import run
from iet_ch5.logging import dump


def capture(local_config, output):
    c = json.loads(local_config.read_text())
    # Trailing body-relative FRD camera, looking towards the aircraft's centre.
    position = np.array([-3.0, -2.0, -1.5])
    yaw = np.arctan2(2.0, 3.0)
    pitch = -np.arctan2(1.5, np.hypot(3.0, 2.0))
    q = Rotation.from_euler("xyz", [0.0, pitch, yaw]).as_quat()
    pose = {
        "position": dict(zip(["x_val", "y_val", "z_val"], position.tolist())),
        "orientation": dict(zip(["x_val", "y_val", "z_val", "w_val"], q.tolist())),
    }
    with RpcClient(c["host"], c["port"]) as rpc:
        rpc.call("simSetCameraPose", "0", pose, c["vehicle"], False)
    result = run(
        c,
        json.loads(Path("configs/controllers.json").read_text()),
        json.loads(Path("configs/tasks.json").read_text()),
        "geometric",
        "circle",
        output,
        capture=True,
    )
    frames = sorted((output / "frames").glob("*.png"))
    if not frames:
        raise RuntimeError("No genuine AirSim frames captured")
    with imageio.get_writer(
        output / "demo.mp4", fps=5, codec="libx264", quality=8, macro_block_size=16
    ) as writer:
        for path in frames:
            writer.append_data(imageio.imread(path))
    (output / "camera_provenance.json").write_text(
        dump(
            {
                "run_id": output.name,
                "controller": "geometric",
                "camera": "0",
                "relative_frame": "FRD",
                "relative_pose": pose,
                "capture": "Actual AirSim Scene frames; separate demo excluded from benchmark timing",
                "video_fps": 5,
                "frame_count": len(frames),
                "video_sha256": hashlib.sha256((output / "demo.mp4").read_bytes()).hexdigest(),
                "image_timestamps": "frames.json",
                "result_status": result.status,
            }
        )
        + "\n"
    )
    print(
        dump(
            {
                "run": output.name,
                "status": result.status,
                "frames": len(frames),
                "video_bytes": (output / "demo.mp4").stat().st_size,
            }
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--local-config", type=Path, required=True)
    p.add_argument("--exclusive-simulator", action="store_true", required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    capture(a.local_config, a.output)

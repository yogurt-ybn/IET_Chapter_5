import json

import numpy as np
import pytest

from iet_ch5.evaluation.reanalyze import reanalyze


def test_exact_interval_includes_terminal_truth_and_handles_redundant_setup(tmp_path):
    from pathlib import Path

    config = json.loads(Path("configs/tasks.json").read_text())
    config.update(duration=0.024, transfer_displacement=[0.0, 0.0, 0.0])
    state = {
        "timestamp_ns": 0,
        "p": [0.2, 0.0, 2.0],
        "v": [0.0, 0.0, 0.0],
        "R": np.eye(3).tolist(),
        "omega": [0.0, 0.0, 0.0],
    }
    metadata = {
        "config": {"task_config": config, "task": "hover_transfer"},
        "verified_initial_state": state,
        "parameters": {"gravity": 9.81},
    }
    (tmp_path / "metadata.json").write_text(json.dumps(metadata))
    vector = {"x_val": 0.0, "y_val": 0.0, "z_val": 0.0}
    k = {
        "position": {"x_val": 0.2, "y_val": 0.0, "z_val": -2.0},
        "linear_velocity": vector,
        "angular_velocity": vector,
        "orientation": {"w_val": 1.0, "x_val": 0.0, "y_val": 0.0, "z_val": 0.0},
    }
    (tmp_path / "physics.jsonl").write_text(
        "\n".join(
            json.dumps({"timestamp": t, "kinematics": k})
            for t in [0, 6_000_000, 15_000_000, 30_000_000]
        )
    )
    reference = {"a": [0, 0, 0], "jerk": [0, 0, 0], "heading": 0.0, "heading_rate": 0.0}
    (tmp_path / "log.jsonl").write_text(
        "\n".join(
            json.dumps(
                {
                    "t": t,
                    "reference": reference,
                    "diagnostics": {},
                    "motor_pwm": [0.6] * 4,
                    "saturation": [False] * 4,
                }
            )
            for t in [0.0, 0.012]
        )
    )
    metrics, data = reanalyze(tmp_path)
    assert metrics["complete_common_interval"]
    assert metrics["valid_interval_s"] == [0.0, 0.024]
    assert data["t"][-1] == 0.024
    assert metrics["redundant_setup_snapshot"]
    assert metrics["position_rms_m"] == pytest.approx(0.2)
    assert metrics["attitude_rms_rad"] == pytest.approx(0.0)

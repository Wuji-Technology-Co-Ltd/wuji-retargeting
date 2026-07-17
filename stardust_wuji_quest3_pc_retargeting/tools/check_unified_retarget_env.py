from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path
from typing import Sequence

import numpy as np

from stardust_wuji_quest3_pc_retargeting.hand_control.retarget_pipeline import (
    RetargetPipeline,
)
from stardust_wuji_quest3_pc_retargeting.runtime.config import load_yaml_config
from stardust_wuji_quest3_pc_retargeting.safety.hand_safety_filter import (
    HandSafetyFilter,
)


FINGER_CHAINS = (
    (1, 2, 3, 4),
    (5, 6, 7, 8),
    (9, 10, 11, 12),
    (13, 14, 15, 16),
    (17, 18, 19, 20),
)


def canonical_mp21(*, closed: bool, mirror_x: bool) -> np.ndarray:
    points = np.zeros((21, 3), dtype=float)
    bases = (
        (-0.035, 0.018),
        (-0.018, 0.055),
        (0.000, 0.060),
        (0.018, 0.055),
        (0.035, 0.045),
    )
    lengths = (
        (0.030, 0.026, 0.022, 0.018),
        (0.045, 0.030, 0.022, 0.018),
        (0.050, 0.033, 0.024, 0.019),
        (0.047, 0.031, 0.023, 0.018),
        (0.040, 0.026, 0.020, 0.016),
    )
    closed_directions = (
        np.asarray((0.0, 0.55, -0.84)),
        np.asarray((0.0, 0.0, -1.0)),
        np.asarray((0.0, -0.65, -0.76)),
        np.asarray((0.0, -0.9, -0.43)),
    )
    for finger, (base_x, base_y) in enumerate(bases):
        position = np.asarray((base_x, base_y, 0.0), dtype=float)
        for joint, landmark in enumerate(FINGER_CHAINS[finger]):
            if closed:
                direction = (
                    np.asarray((-0.25, 0.1, -0.96))
                    if finger == 0
                    else closed_directions[joint]
                )
            else:
                direction = (
                    np.asarray((-0.65, 0.75, 0.0))
                    if finger == 0
                    else np.asarray((0.0, 1.0, 0.0))
                )
            direction = direction / np.linalg.norm(direction)
            position = position + direction * lengths[finger][joint]
            points[landmark] = position
    if mirror_x:
        points[:, 0] *= -1.0
    return points


def _settled_output(pipeline: RetargetPipeline, points: np.ndarray) -> np.ndarray:
    output = np.zeros(20, dtype=float)
    for _ in range(20):
        output = np.asarray(pipeline.retarget(points), dtype=float).reshape(20)
    return output


def validate_retargeters(service_config: str) -> dict:
    service = load_yaml_config(service_config)
    configured_hands = service.get("hands", {})
    results = {}
    for side in ("left", "right"):
        hand = configured_hands[side]
        pipeline = RetargetPipeline(
            config_path=hand["retarget_config"],
            hand_side=side,
            dry_run=False,
        )
        safety = HandSafetyFilter.from_yaml(hand["safety_config"])
        open_qpos = _settled_output(
            pipeline,
            canonical_mp21(closed=False, mirror_x=side == "left"),
        )
        closed_qpos = _settled_output(
            pipeline,
            canonical_mp21(closed=True, mirror_x=side == "left"),
        )
        open_safe = np.asarray(
            safety.filter(
                open_qpos,
                frame_age_sec=0.0,
                deadman=True,
                tracking_valid=True,
            ).qpos,
            dtype=float,
        )
        closed_safe = np.asarray(
            safety.filter(
                closed_qpos,
                frame_age_sec=0.0,
                deadman=True,
                tracking_valid=True,
            ).qpos,
            dtype=float,
        )
        finite = bool(
            np.isfinite(open_qpos).all()
            and np.isfinite(closed_qpos).all()
            and np.isfinite(open_safe).all()
            and np.isfinite(closed_safe).all()
        )
        delta_l2 = float(np.linalg.norm(closed_qpos - open_qpos))
        results[side] = {
            "retarget_config": hand["retarget_config"],
            "safety_config": hand["safety_config"],
            "finite": finite,
            "raw_open_min": float(open_qpos.min()),
            "raw_open_max": float(open_qpos.max()),
            "raw_closed_min": float(closed_qpos.min()),
            "raw_closed_max": float(closed_qpos.max()),
            "safe_open_min": float(open_safe.min()),
            "safe_open_max": float(open_safe.max()),
            "safe_closed_min": float(closed_safe.min()),
            "safe_closed_max": float(closed_safe.max()),
            "open_closed_delta_l2": delta_l2,
            "passed": finite and delta_l2 > 0.25,
        }
    return results


def check_imports(sdk_root: str) -> dict:
    versions = {}
    for module_name in (
        "numpy",
        "scipy",
        "nlopt",
        "pinocchio",
        "yaml",
        "websockets",
        "rclpy",
    ):
        module = importlib.import_module(module_name)
        versions[module_name] = {
            "version": str(getattr(module, "__version__", "")),
            "path": str(getattr(module, "__file__", "")),
        }

    from stardust_wuji_quest3_pc_retargeting.hardware_audit.m7_audit import (
        prepare_vendor_robotics_import,
    )

    prepare_vendor_robotics_import(sdk_root)
    module = importlib.import_module(
        "astribot_sdk.core.astribot_api.astribot_client"
    )
    versions["astribot_sdk"] = {
        "version": "",
        "path": str(getattr(module, "__file__", "")),
        "factory": str(module.Astribot),
    }
    return versions


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Verify the unified Astribot/real-Wuji-Retargeter environment without "
            "connecting to hand hardware."
        )
    )
    parser.add_argument(
        "--config",
        default="configs/services/control_pc_default.yaml",
    )
    parser.add_argument(
        "--sdk-root",
        default="/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master",
    )
    parser.add_argument("--json-output", default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    report = {
        "schema": "wuji_unified_retarget_env_check.v1",
        "python": sys.executable,
        "imports": check_imports(args.sdk_root),
        "hands": validate_retargeters(args.config),
        "recording_only": True,
        "driver_commands_published": False,
    }
    report["passed"] = all(
        hand["passed"] for hand in report["hands"].values()
    )
    encoded = json.dumps(report, indent=2, sort_keys=True)
    print(encoded)
    if args.json_output:
        output = Path(args.json_output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

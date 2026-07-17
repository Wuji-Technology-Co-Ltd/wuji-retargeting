from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


def analyze_mapping_snapshots(
    opened: Mapping[str, Sequence[float]],
    left_closed: Mapping[str, Sequence[float]],
    right_closed: Mapping[str, Sequence[float]],
) -> dict:
    arrays = {
        phase: {
            side: np.asarray(values[side], dtype=float).reshape(5, 4)
            for side in ("left", "right")
        }
        for phase, values in (
            ("open", opened),
            ("left_closed", left_closed),
            ("right_closed", right_closed),
        )
    }
    left_delta = arrays["left_closed"]["left"] - arrays["open"]["left"]
    left_cross = arrays["left_closed"]["right"] - arrays["open"]["right"]
    right_delta = arrays["right_closed"]["right"] - arrays["open"]["right"]
    right_cross = arrays["right_closed"]["left"] - arrays["open"]["left"]

    def side_report(delta: np.ndarray, cross: np.ndarray) -> dict:
        active_l2 = float(np.linalg.norm(delta))
        cross_l2 = float(np.linalg.norm(cross))
        flexion = delta[1:, 2:4]
        return {
            "active_delta_l2": active_l2,
            "inactive_hand_delta_l2": cross_l2,
            "inactive_to_active_ratio": (
                None if active_l2 <= 1e-9 else cross_l2 / active_l2
            ),
            "positive_flexion_joints": int(np.count_nonzero(flexion > 0.05)),
            "expected_flexion_joints": int(flexion.size),
            "per_finger_delta_l2": {
                name: float(np.linalg.norm(delta[index]))
                for index, name in enumerate(FINGER_NAMES)
            },
            "delta_matrix": delta.astype(float).tolist(),
            "responsive": active_l2 > 0.25,
        }

    return {
        "left": side_report(left_delta, left_cross),
        "right": side_report(right_delta, right_cross),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Interactively verify left/right real-retarget outputs on recording-only "
            "ROS2 topics. No driver command topics are published by this tool."
        )
    )
    parser.add_argument("--capture-sec", type=float, default=0.75)
    parser.add_argument("--wait-timeout-sec", type=float, default=10.0)
    parser.add_argument("--json-output", default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.capture_sec <= 0.0 or args.wait_timeout_sec <= 0.0:
        raise SystemExit("capture and wait timeouts must be positive")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )

    class InspectorNode(Node):
        def __init__(self) -> None:
            super().__init__("quest3_wujihand_mapping_inspector")
            self.samples = {
                side: deque(maxlen=500) for side in ("left", "right")
            }
            self.bridge_state = None
            for side in ("left", "right"):
                self.create_subscription(
                    JointState,
                    f"/teleop/hand/{side}/target_safe",
                    lambda message, selected=side: self._on_target(selected, message),
                    sensor_qos,
                )
            self.create_subscription(
                String,
                "/teleop/hand/bridge_state",
                self._on_state,
                10,
            )

        def _on_target(self, side: str, message) -> None:
            values = np.asarray(message.position, dtype=float)
            if values.size == 20 and np.isfinite(values).all():
                self.samples[side].append((time.monotonic(), values.copy()))

        def _on_state(self, message) -> None:
            try:
                self.bridge_state = json.loads(message.data)
            except (TypeError, ValueError):
                self.bridge_state = None

        def capture(self, duration: float) -> dict[str, list[float]]:
            started = time.monotonic()
            time.sleep(duration)
            captured = {}
            for side in ("left", "right"):
                rows = [
                    values
                    for timestamp, values in self.samples[side]
                    if timestamp >= started
                ]
                if not rows:
                    raise RuntimeError(f"no {side} target samples captured")
                captured[side] = np.median(np.stack(rows), axis=0).tolist()
            return captured

    rclpy.init(args=None)
    node = InspectorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + args.wait_timeout_sec
        while time.monotonic() < deadline:
            if (
                node.samples["left"]
                and node.samples["right"]
                and node.bridge_state is not None
            ):
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("timed out waiting for recording-only hand topics")

        if bool(node.bridge_state.get("driver_commands_enabled")):
            raise RuntimeError("bridge driver commands are enabled; inspection aborted")
        if node.bridge_state.get("teleop_state") != "RUNNING" or not all(
            bool(node.bridge_state.get("command_enabled", {}).get(side))
            for side in ("left", "right")
        ):
            raise RuntimeError(
                "both hands must be ACTIVE in teleop RUNNING; press E and retry"
            )
        for side in ("left", "right"):
            topic = f"/{side}_hand/joint_commands"
            if node.count_publishers(topic) > 0:
                raise RuntimeError(f"publisher detected on {topic}; inspection aborted")

        input("双手完全张开并保持稳定，然后按 Enter 采样：")
        opened = node.capture(args.capture_sec)
        input("只握紧左手，右手保持张开，然后按 Enter 采样：")
        left_closed = node.capture(args.capture_sec)
        input("重新张开左手，只握紧右手，然后按 Enter 采样：")
        right_closed = node.capture(args.capture_sec)

        analysis = analyze_mapping_snapshots(opened, left_closed, right_closed)
        report = {
            "schema": "quest3_wujihand_mapping_inspection.v1",
            "recording_only": True,
            "driver_commands_enabled": False,
            "bridge_state": node.bridge_state,
            "snapshots": {
                "open": opened,
                "left_closed": left_closed,
                "right_closed": right_closed,
            },
            "analysis": analysis,
        }
        encoded = json.dumps(report, indent=2, sort_keys=True)
        print(encoded)
        if args.json_output:
            output = Path(args.json_output).expanduser()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(encoded + "\n", encoding="utf-8")
        if not all(side["responsive"] for side in analysis.values()):
            print("WARNING: at least one hand did not show a clear open/close response")
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import time
from copy import deepcopy
from typing import Sequence

import websockets

from stardust_wuji_quest3_pc_retargeting.conversion.hand_joint_names import WEBXR_HAND_JOINT_NAMES
from stardust_wuji_quest3_pc_retargeting.protocol.json_codec import encode_message
from stardust_wuji_quest3_pc_retargeting.protocol.messages import SCHEMA


def build_mock_frame(seq: int = 0, t: float | None = None) -> dict:
    now = time.monotonic() if t is None else float(t)
    positions = [[0.01 * i, 0.02 * (i % 5), 0.01 * (seq % 10)] for i, _ in enumerate(WEBXR_HAND_JOINT_NAMES)]
    hand = {
        "valid": True,
        "joint_names": WEBXR_HAND_JOINT_NAMES,
        "positions": positions,
        "orientations_xyzw": [[0.0, 0.0, 0.0, 1.0]] * len(WEBXR_HAND_JOINT_NAMES),
    }
    return {
        "schema": SCHEMA,
        "type": "tracking_frame",
        "seq": int(seq),
        "client_time_sec": now,
        "xr_session_id": "mock",
        "hmd": {"valid": True, "position": [0.0, 1.6, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "hands": {"left": hand, "right": deepcopy(hand)},
        "session": {
            "active": True,
            "visibility": "visible",
            "reference_space": "local-floor",
            "reference_space_revision": 0,
        },
    }


async def send_mock_frames(
    target: str,
    rate_hz: float = 30.0,
    count: int | None = None,
    status_interval_sec: float = 2.0,
) -> None:
    delay = 1.0 / float(rate_hz)
    async with websockets.connect(target) as socket:
        last_status_time = 0.0

        async def receive_states() -> None:
            nonlocal last_status_time
            async for raw in socket:
                message = json.loads(raw)
                now = time.monotonic()
                if message.get("type") in {"control_error", "command_result"} or now - last_status_time >= status_interval_sec:
                    summary = message.get("message") or message.get("teleop_state") or message.get("state") or message.get("type")
                    print(f"[mock] control: {summary}", flush=True)
                    last_status_time = now

        receiver = asyncio.create_task(receive_states())
        seq = 0
        try:
            while count is None or seq < count:
                await socket.send(encode_message(build_mock_frame(seq)))
                seq += 1
                await asyncio.sleep(delay)
        finally:
            receiver.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await receiver


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send mock Quest3 WebXR tracking frames.")
    parser.add_argument("--target", default="ws://127.0.0.1:9001")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--count", type=int, default=None)
    parser.add_argument("--status-interval-sec", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    asyncio.run(send_mock_frames(args.target, args.rate_hz, args.count, args.status_interval_sec))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

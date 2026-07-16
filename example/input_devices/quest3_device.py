from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from stardust_wuji_quest3_pc_retargeting.conversion.webxr_to_mp21 import WebXRToMP21Converter
from stardust_wuji_quest3_pc_retargeting.protocol.validation import ProtocolError, validate_tracking_frame


@dataclass(frozen=True)
class Quest3ControllerState:
    deadman: bool
    connected: bool
    left_valid: bool
    right_valid: bool
    frame_age_sec: float | None
    seq: int | None


class Quest3Device:
    """WebSocket input device that matches `example/teleop_sim.py` expectations."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9001,
        left_config: str | None = None,
        right_config: str | None = None,
        grip_deadman_threshold: float = 0.5,
        stale_timeout_sec: float = 0.2,
        start_server: bool = True,
    ):
        self.host = host
        self.port = int(port)
        self.left_config = left_config
        self.right_config = right_config
        self.grip_deadman_threshold = float(grip_deadman_threshold)
        self.stale_timeout_sec = float(stale_timeout_sec)
        self.left_converter = self._load_converter(left_config)
        self.right_converter = self._load_converter(right_config)
        self._lock = threading.Lock()
        self._latest = None
        self._received_monotonic: float | None = None
        self._last_error = ""
        self._loop: asyncio.AbstractEventLoop | None = None
        self._server = None
        self._thread: threading.Thread | None = None
        self._started = threading.Event()
        self._closed = False
        if start_server:
            self.start()

    @classmethod
    def from_service_config(cls, path: str | Path, start_server: bool = True) -> "Quest3Device":
        cfg_path = Path(path).expanduser()
        data = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        quest_web = data.get("quest_web", {}) if isinstance(data.get("quest_web"), dict) else {}
        hands = data.get("hands", {}) if isinstance(data.get("hands"), dict) else {}
        left = hands.get("left", {}) if isinstance(hands.get("left"), dict) else {}
        right = hands.get("right", {}) if isinstance(hands.get("right"), dict) else {}

        def resolve(value: str | None) -> str | None:
            if not value:
                return None
            candidate = Path(value).expanduser()
            if candidate.is_absolute():
                return str(candidate)
            direct = (cfg_path.parent / candidate).resolve()
            if direct.exists():
                return str(direct)
            return str((PROJECT_ROOT / candidate).resolve())

        return cls(
            host=quest_web.get("listen_host", data.get("quest_host", "0.0.0.0")),
            port=int(quest_web.get("listen_port", data.get("quest_port", 9001))),
            left_config=resolve(left.get("mapping_config", data.get("quest_left_config"))),
            right_config=resolve(right.get("mapping_config", data.get("quest_right_config"))),
            grip_deadman_threshold=float(data.get("grip_deadman_threshold", 0.5)),
            stale_timeout_sec=float(quest_web.get("stale_timeout_sec", data.get("stale_timeout_sec", 0.2))),
            start_server=start_server,
        )

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run_server_thread, daemon=True)
        self._thread.start()
        if not self._started.wait(timeout=2.0):
            raise RuntimeError(f"Quest3Device WebSocket server did not start on {self.host}:{self.port}")

    def ingest_payload(self, payload: dict[str, Any]) -> None:
        frame = validate_tracking_frame(payload)
        with self._lock:
            self._latest = frame
            self._received_monotonic = time.monotonic()
            self._last_error = ""

    def get_fingers_data(self) -> dict[str, np.ndarray]:
        zeros = np.zeros((21, 3), dtype=float)
        with self._lock:
            frame = self._latest
            received = self._received_monotonic
        if frame is None or received is None or time.monotonic() - received > self.stale_timeout_sec:
            left = zeros.copy()
            right = zeros.copy()
        else:
            left = self.left_converter.convert(frame.hands["left"])
            right = self.right_converter.convert(frame.hands["right"])
        return {
            "left_fingers": left,
            "right_fingers": right,
            "left": left,
            "right": right,
        }

    def get_frame_age_sec(self) -> float | None:
        with self._lock:
            received = self._received_monotonic
        if received is None:
            return None
        return max(0.0, time.monotonic() - received)

    def get_controller_state(self) -> Quest3ControllerState:
        with self._lock:
            frame = self._latest
            received = self._received_monotonic
        age = None if received is None else max(0.0, time.monotonic() - received)
        fresh = age is not None and age <= self.stale_timeout_sec
        left_valid = bool(frame and frame.hands["left"].valid and fresh)
        right_valid = bool(frame and frame.hands["right"].valid and fresh)
        return Quest3ControllerState(
            deadman=bool(frame and frame.session.active and frame.hmd.valid and fresh and (left_valid or right_valid)),
            connected=bool(fresh),
            left_valid=left_valid,
            right_valid=right_valid,
            frame_age_sec=age,
            seq=None if frame is None else frame.seq,
        )

    def close(self) -> None:
        self._closed = True
        if self._loop and self._server:
            self._loop.call_soon_threadsafe(self._server.close)
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    stop = close
    cleanup = close

    def _load_converter(self, config_path: str | None) -> WebXRToMP21Converter:
        if not config_path:
            return WebXRToMP21Converter()
        path = Path(config_path).expanduser()
        if not path.is_absolute():
            path = (PROJECT_ROOT / path).resolve()
        if not path.exists():
            return WebXRToMP21Converter()
        return WebXRToMP21Converter.from_yaml(path)

    def _run_server_thread(self) -> None:
        import websockets

        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)

        async def handler(socket, *_args):
            async for raw in socket:
                try:
                    payload = json.loads(raw)
                    if payload.get("type") != "tracking_frame":
                        continue
                    self.ingest_payload(payload)
                except (json.JSONDecodeError, ProtocolError, ValueError) as exc:
                    with self._lock:
                        self._last_error = str(exc)

        async def serve():
            self._server = await websockets.serve(handler, self.host, self.port)
            self._started.set()

        loop.run_until_complete(serve())
        try:
            loop.run_forever()
        finally:
            if self._server is not None:
                self._server.close()
                loop.run_until_complete(self._server.wait_closed())
            loop.close()

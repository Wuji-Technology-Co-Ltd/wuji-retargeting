from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import socket
from threading import Lock
from typing import Mapping, Protocol

import numpy as np


BRIDGE_SCHEMA = "quest3_wujihand_bridge.v1"


@dataclass(frozen=True)
class HandBridgeSide:
    valid: bool
    mp21: list[list[float]]
    raw_qpos: list[float]
    safe_qpos: list[float]
    enabled: bool
    safety_state: str
    reason: str = ""

    def validate(self) -> None:
        mp21 = np.asarray(self.mp21, dtype=float)
        raw = np.asarray(self.raw_qpos, dtype=float)
        safe = np.asarray(self.safe_qpos, dtype=float)
        if mp21.shape != (21, 3) or not np.isfinite(mp21).all():
            raise ValueError("hand bridge mp21 must be finite with shape (21, 3)")
        if raw.shape != (20,) or not np.isfinite(raw).all():
            raise ValueError("hand bridge raw_qpos must contain 20 finite values")
        if safe.shape != (20,) or not np.isfinite(safe).all():
            raise ValueError("hand bridge safe_qpos must contain 20 finite values")


@dataclass(frozen=True)
class HandBridgeFrame:
    seq: int
    client_time_sec: float
    receive_time_ns: int
    xr_session_id: str
    teleop_state: str
    hands: dict[str, HandBridgeSide]
    schema: str = BRIDGE_SCHEMA
    type: str = "hand_command_frame"

    def validate(self) -> None:
        if self.schema != BRIDGE_SCHEMA or self.type != "hand_command_frame":
            raise ValueError("hand bridge schema/type is invalid")
        if int(self.seq) < 0 or int(self.receive_time_ns) < 0:
            raise ValueError("hand bridge seq and receive_time_ns must be non-negative")
        if not np.isfinite(float(self.client_time_sec)):
            raise ValueError("hand bridge client_time_sec must be finite")
        if set(self.hands) != {"left", "right"}:
            raise ValueError("hand bridge frame requires left and right hands")
        for side in ("left", "right"):
            self.hands[side].validate()

    def to_dict(self) -> dict:
        self.validate()
        return asdict(self)

    def to_json_bytes(self) -> bytes:
        return json.dumps(
            self.to_dict(),
            ensure_ascii=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")

    @classmethod
    def from_dict(cls, data: Mapping) -> "HandBridgeFrame":
        raw_hands = data.get("hands", {})
        hands = {
            side: HandBridgeSide(**dict(raw_hands[side]))
            for side in ("left", "right")
        }
        frame = cls(
            seq=int(data["seq"]),
            client_time_sec=float(data["client_time_sec"]),
            receive_time_ns=int(data["receive_time_ns"]),
            xr_session_id=str(data.get("xr_session_id", "")),
            teleop_state=str(data.get("teleop_state", "IDLE")),
            hands=hands,
            schema=str(data.get("schema", "")),
            type=str(data.get("type", "")),
        )
        frame.validate()
        return frame

    @classmethod
    def from_json_bytes(cls, payload: bytes) -> "HandBridgeFrame":
        return cls.from_dict(json.loads(payload.decode("utf-8")))


class HandCommandSink(Protocol):
    @property
    def name(self) -> str: ...

    def publish(self, frame: HandBridgeFrame) -> None: ...

    def close(self) -> None: ...


class DryRunHandCommandSink:
    def __init__(self, history_size: int = 100):
        self.history_size = max(1, int(history_size))
        self._lock = Lock()
        self.last_frame: HandBridgeFrame | None = None
        self.history: list[HandBridgeFrame] = []
        self.publish_count = 0

    @property
    def name(self) -> str:
        return "dry-run"

    def publish(self, frame: HandBridgeFrame) -> None:
        frame.validate()
        with self._lock:
            self.last_frame = frame
            self.history.append(frame)
            if len(self.history) > self.history_size:
                del self.history[: len(self.history) - self.history_size]
            self.publish_count += 1

    def snapshot(self) -> HandBridgeFrame | None:
        with self._lock:
            return self.last_frame

    def close(self) -> None:
        return None


class UdpHandCommandSink:
    def __init__(self, host: str = "127.0.0.1", port: int = 9011):
        self.host = str(host)
        self.port = int(port)
        if not 1 <= self.port <= 65535:
            raise ValueError("hand bridge UDP port must be in [1, 65535]")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.publish_count = 0
        self.last_error = ""

    @property
    def name(self) -> str:
        return f"udp://{self.host}:{self.port}"

    def publish(self, frame: HandBridgeFrame) -> None:
        payload = frame.to_json_bytes()
        if len(payload) > 60_000:
            raise RuntimeError("hand bridge frame exceeds safe UDP datagram size")
        try:
            self._socket.sendto(payload, (self.host, self.port))
        except OSError as exc:
            self.last_error = str(exc)
            raise
        self.publish_count += 1
        self.last_error = ""

    def close(self) -> None:
        self._socket.close()

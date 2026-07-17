from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

from .command_bridge import HandBridgeFrame


@dataclass
class Ros2BridgeStats:
    accepted_frames: int = 0
    rejected_frames: int = 0
    duplicate_frames: int = 0
    out_of_order_frames: int = 0


class Ros2BridgeCore:
    """Transport validation shared by the ROS2 bridge and unit tests."""

    def __init__(self, command_timeout_sec: float = 0.25):
        self.command_timeout_sec = float(command_timeout_sec)
        if not 0.05 <= self.command_timeout_sec <= 5.0:
            raise ValueError("hand bridge command timeout must be in [0.05, 5.0] seconds")
        self.stats = Ros2BridgeStats()
        self.last_frame: HandBridgeFrame | None = None
        self.last_receive_monotonic: float | None = None

    def ingest(self, payload: bytes, now_monotonic: float | None = None) -> HandBridgeFrame | None:
        try:
            frame = HandBridgeFrame.from_json_bytes(payload)
        except Exception:
            self.stats.rejected_frames += 1
            raise
        previous = self.last_frame
        if previous is not None and frame.xr_session_id == previous.xr_session_id:
            if frame.seq == previous.seq:
                self.stats.duplicate_frames += 1
                return None
            if frame.seq < previous.seq:
                self.stats.out_of_order_frames += 1
                return None
        self.last_frame = frame
        self.last_receive_monotonic = monotonic() if now_monotonic is None else float(now_monotonic)
        self.stats.accepted_frames += 1
        return frame

    def stale(self, now_monotonic: float | None = None) -> bool:
        if self.last_receive_monotonic is None:
            return True
        now = monotonic() if now_monotonic is None else float(now_monotonic)
        return now - self.last_receive_monotonic > self.command_timeout_sec

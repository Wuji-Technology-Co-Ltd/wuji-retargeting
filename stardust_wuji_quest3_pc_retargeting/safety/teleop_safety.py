from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Freshness:
    fresh: bool
    disabled: bool
    age_sec: float


class FrameFreshnessMonitor:
    def __init__(self, fresh_timeout_sec: float = 0.05, disable_timeout_sec: float = 0.10):
        self.fresh_timeout_sec = float(fresh_timeout_sec)
        self.disable_timeout_sec = float(disable_timeout_sec)
        self.last_frame_time: float | None = None

    def update(self, now_sec: float) -> None:
        self.last_frame_time = float(now_sec)

    def update_ns(self, receive_time_ns: int) -> None:
        self.update(int(receive_time_ns) / 1e9)

    def check_ns(self, now_ns: int) -> Freshness:
        return self.check(int(now_ns) / 1e9)

    def check(self, now_sec: float) -> Freshness:
        if self.last_frame_time is None:
            return Freshness(fresh=False, disabled=True, age_sec=float("inf"))
        age = max(0.0, float(now_sec) - self.last_frame_time)
        return Freshness(
            fresh=age <= self.fresh_timeout_sec,
            disabled=age > self.disable_timeout_sec,
            age_sec=age,
        )

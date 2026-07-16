from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
import yaml


class SafetyState(str, Enum):
    ACTIVE = "ACTIVE"
    HOLD = "HOLD"
    SAFE_OPEN = "SAFE_OPEN"
    DISABLED = "DISABLED"


@dataclass
class HandCommand:
    qpos: list[float]
    enabled: bool
    reason: str = ""
    state: SafetyState = SafetyState.HOLD


class HandSafetyFilter:
    def __init__(
        self,
        lower=None,
        upper=None,
        max_delta: float = 0.35,
        stale_timeout_sec: float = 0.2,
        disable_timeout_sec: float = 1.0,
        safe_open=None,
        safe_open_on_deadman_release: bool = False,
    ):
        self.lower = np.asarray(lower if lower is not None else [-1.0] * 20, dtype=float)
        self.upper = np.asarray(upper if upper is not None else [1.0] * 20, dtype=float)
        self.max_delta = float(max_delta)
        self.stale_timeout_sec = float(stale_timeout_sec)
        self.disable_timeout_sec = float(disable_timeout_sec)
        self.safe_open = np.asarray(safe_open if safe_open is not None else [0.0] * 20, dtype=float)
        self.safe_open_on_deadman_release = bool(safe_open_on_deadman_release)
        self._last_qpos: np.ndarray | None = None
        self._last_valid_time: float | None = None

    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
        safe_open_on_deadman_release: bool = False,
    ) -> "HandSafetyFilter":
        config_path = Path(path).expanduser()
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        joint_limits = data.get("joint_limits", {}) if isinstance(data.get("joint_limits"), dict) else {}
        return cls(
            lower=joint_limits.get("lower"),
            upper=joint_limits.get("upper"),
            max_delta=data.get("max_qpos_jump", data.get("max_delta", 0.35)),
            stale_timeout_sec=data.get("fresh_timeout_sec", data.get("stale_timeout_sec", 0.2)),
            disable_timeout_sec=data.get("disable_timeout_sec", 1.0),
            safe_open=data.get("safe_open_qpos"),
            safe_open_on_deadman_release=safe_open_on_deadman_release,
        )

    def filter(
        self,
        qpos,
        valid: bool | None = None,
        now_sec: float | None = None,
        running: bool | None = None,
        *,
        frame_age_sec: float | None = None,
        deadman: bool | None = None,
        tracking_valid: bool | None = None,
    ) -> HandCommand:
        if frame_age_sec is not None or deadman is not None or tracking_valid is not None:
            return self._filter_teleop(
                qpos=qpos,
                frame_age_sec=frame_age_sec,
                deadman=bool(deadman),
                tracking_valid=bool(tracking_valid),
            )
        if valid is None or now_sec is None or running is None:
            raise TypeError("filter requires either legacy valid/now_sec/running or teleop keyword arguments")
        candidate = np.asarray(qpos, dtype=float).reshape(-1)
        if candidate.size != 20 or not np.isfinite(candidate).all():
            return self._hold(False, "invalid qpos", SafetyState.HOLD)
        if not running:
            return self._hold(False, "not running", SafetyState.HOLD)
        if not valid:
            return self._hold(False, "tracking invalid", SafetyState.HOLD)
        if self._last_valid_time is not None and now_sec - self._last_valid_time > self.stale_timeout_sec:
            return self._hold(False, "stale", SafetyState.DISABLED)
        clipped = np.clip(candidate, self.lower, self.upper)
        if self._last_qpos is not None:
            clipped = self._last_qpos + np.clip(clipped - self._last_qpos, -self.max_delta, self.max_delta)
        self._last_qpos = clipped
        self._last_valid_time = float(now_sec)
        return HandCommand(qpos=clipped.astype(float).tolist(), enabled=True, state=SafetyState.ACTIVE)

    def _filter_teleop(
        self,
        qpos,
        frame_age_sec: float | None,
        deadman: bool,
        tracking_valid: bool,
    ) -> HandCommand:
        if frame_age_sec is None or frame_age_sec > self.disable_timeout_sec:
            return self._hold(False, "stale", SafetyState.DISABLED)
        if frame_age_sec > self.stale_timeout_sec:
            return self._hold(False, "stale", SafetyState.DISABLED)
        if not tracking_valid:
            return self._hold(False, "tracking invalid", SafetyState.HOLD)
        if not deadman:
            if self.safe_open_on_deadman_release:
                qpos_list = self.safe_open.astype(float).tolist()
                return HandCommand(qpos=qpos_list, enabled=True, reason="deadman released", state=SafetyState.SAFE_OPEN)
            return self._hold(False, "deadman released", SafetyState.HOLD)
        return self.filter(qpos, valid=True, now_sec=0.0, running=True)

    def _hold(self, enabled: bool, reason: str, state: SafetyState) -> HandCommand:
        qpos = self._last_qpos if self._last_qpos is not None else self.safe_open
        return HandCommand(
            qpos=np.asarray(qpos, dtype=float).astype(float).tolist(),
            enabled=enabled,
            reason=reason,
            state=state,
        )

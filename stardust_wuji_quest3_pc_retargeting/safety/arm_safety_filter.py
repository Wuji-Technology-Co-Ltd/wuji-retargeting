from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmTarget
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import (
    align_quat_sign_xyzw,
    normalize_quat_xyzw,
    quat_angle_xyzw,
    quat_slerp_xyzw,
)


class ArmSafetyState(str, Enum):
    ACTIVE = "ACTIVE"
    HOLD = "HOLD"
    PAUSED = "PAUSED"
    FAULT = "FAULT"


@dataclass
class ArmCommand:
    target: ArmTarget
    enabled: bool
    reason: str = ""
    state: ArmSafetyState = ArmSafetyState.HOLD
    workspace_clipped: bool = False


class ArmSafetyFilter:
    def __init__(
        self,
        xyz_min=None,
        xyz_max=None,
        max_linear_speed_mps: float = 0.10,
        max_angular_speed_rad_s: float = 0.50,
        max_input_position_jump_m: float = 0.10,
        max_input_rotation_jump_rad: float = 0.80,
        minimum_dt_sec: float = 0.002,
        maximum_dt_sec: float = 0.050,
        position_alpha: float = 1.0,
        orientation_alpha: float = 1.0,
        max_position_delta: float | None = None,
    ):
        self.xyz_min = np.asarray(xyz_min if xyz_min is not None else [-2.0, -2.0, -2.0], dtype=float)
        self.xyz_max = np.asarray(xyz_max if xyz_max is not None else [2.0, 2.0, 2.0], dtype=float)
        if self.xyz_min.shape != (3,) or self.xyz_max.shape != (3,) or np.any(self.xyz_min > self.xyz_max):
            raise ValueError("workspace bounds must be valid 3-vectors")
        self.max_linear_speed_mps = float(max_linear_speed_mps)
        self.max_angular_speed_rad_s = float(max_angular_speed_rad_s)
        self.max_input_position_jump_m = float(max_input_position_jump_m)
        self.max_input_rotation_jump_rad = float(max_input_rotation_jump_rad)
        self.minimum_dt_sec = float(minimum_dt_sec)
        self.maximum_dt_sec = float(maximum_dt_sec)
        self.position_alpha = float(position_alpha)
        self.orientation_alpha = float(orientation_alpha)
        self._compatibility_step = None if max_position_delta is None else float(max_position_delta)
        if not (0.0 < self.minimum_dt_sec <= self.maximum_dt_sec):
            raise ValueError("dt bounds must be positive and ordered")
        if any(value <= 0.0 for value in (self.max_linear_speed_mps, self.max_angular_speed_rad_s)):
            raise ValueError("speed limits must be positive")
        if not (0.0 < self.position_alpha <= 1.0 and 0.0 < self.orientation_alpha <= 1.0):
            raise ValueError("filter alpha values must be in (0, 1]")
        self._last_raw: ArmTarget | None = None
        self._last_filtered: ArmTarget | None = None
        self._last_target: ArmTarget | None = None

    def reset(self, target: ArmTarget | None = None) -> None:
        copied = None if target is None else self._copy_target(target)
        self._last_raw = copied
        self._last_filtered = copied
        self._last_target = copied

    def filter(
        self,
        target: ArmTarget,
        valid: bool,
        running: bool,
        dt_sec: float | None = None,
        paused: bool = False,
        fault: bool = False,
    ) -> ArmCommand:
        if fault:
            return self._hold(ArmSafetyState.FAULT, "control loop fault")
        if paused:
            return self._hold(ArmSafetyState.PAUSED, "teleop paused")
        if not running:
            return self._hold(ArmSafetyState.HOLD, "not running")
        if not valid:
            return self._hold(ArmSafetyState.HOLD, "tracking invalid")
        try:
            raw = self._copy_target(target)
        except (TypeError, ValueError):
            return self._hold(ArmSafetyState.FAULT, "invalid target")
        if self._last_raw is not None:
            position_jump = float(np.linalg.norm(raw.position_array() - self._last_raw.position_array()))
            rotation_jump = quat_angle_xyzw(raw.orientation_array(), self._last_raw.orientation_array())
            if position_jump > self.max_input_position_jump_m:
                return self._hold(ArmSafetyState.HOLD, f"input position jump {position_jump:.6f} m")
            if rotation_jump > self.max_input_rotation_jump_rad:
                return self._hold(ArmSafetyState.HOLD, f"input rotation jump {rotation_jump:.6f} rad")
        self._last_raw = raw
        if self._last_target is None:
            position = np.clip(raw.position_array(), self.xyz_min, self.xyz_max)
            workspace_clipped = not np.allclose(position, raw.position_array())
            accepted = ArmTarget(position.tolist(), raw.orientation_array().tolist())
            self._last_filtered = accepted
            self._last_target = accepted
            return ArmCommand(accepted, True, state=ArmSafetyState.ACTIVE, workspace_clipped=workspace_clipped)

        try:
            dt = self._resolve_dt(dt_sec)
        except ValueError as exc:
            return self._hold(ArmSafetyState.FAULT, str(exc))
        filtered_position = self._last_filtered.position_array() + self.position_alpha * (
            raw.position_array() - self._last_filtered.position_array()
        )
        filtered_orientation = quat_slerp_xyzw(
            self._last_filtered.orientation_array(), raw.orientation_array(), self.orientation_alpha
        )
        self._last_filtered = ArmTarget(filtered_position.tolist(), filtered_orientation.tolist())
        workspace_position = np.clip(filtered_position, self.xyz_min, self.xyz_max)
        workspace_clipped = not np.allclose(workspace_position, filtered_position)
        previous_position = self._last_target.position_array()
        delta = workspace_position - previous_position
        distance = float(np.linalg.norm(delta))
        max_step = self._compatibility_step if self._compatibility_step is not None and dt_sec is None else self.max_linear_speed_mps * dt
        if distance > max_step:
            delta *= max_step / distance
        position = previous_position + delta
        previous_orientation = self._last_target.orientation_array()
        filtered_orientation = align_quat_sign_xyzw(filtered_orientation, previous_orientation)
        angle = quat_angle_xyzw(previous_orientation, filtered_orientation)
        max_angle = self.max_angular_speed_rad_s * dt
        orientation = (
            quat_slerp_xyzw(previous_orientation, filtered_orientation, max_angle / angle)
            if angle > max_angle and angle > 0.0
            else filtered_orientation
        )
        accepted = ArmTarget(position.tolist(), normalize_quat_xyzw(orientation).tolist())
        self._last_target = accepted
        return ArmCommand(accepted, True, state=ArmSafetyState.ACTIVE, workspace_clipped=workspace_clipped)

    def _resolve_dt(self, dt_sec: float | None) -> float:
        if dt_sec is None:
            return self.maximum_dt_sec
        dt = float(dt_sec)
        if not np.isfinite(dt) or dt <= 0.0 or dt > self.maximum_dt_sec:
            raise ValueError(f"invalid control dt {dt!r}")
        return max(dt, self.minimum_dt_sec)

    def _hold(self, state: ArmSafetyState, reason: str) -> ArmCommand:
        target = self._last_target or ArmTarget([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        return ArmCommand(self._copy_target(target), False, reason=reason, state=state)

    @staticmethod
    def _copy_target(target: ArmTarget) -> ArmTarget:
        if not isinstance(target, ArmTarget):
            raise ValueError("target must be an ArmTarget")
        return ArmTarget(target.position_array().tolist(), target.orientation_array().tolist())

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Mapping

import numpy as np

from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import AbsoluteCalibrationResult
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import (
    align_quat_sign_xyzw,
    matrix_to_quat_xyzw,
    normalize_quat_xyzw,
    quat_angle_xyzw,
    quat_inverse_xyzw,
    quat_multiply_xyzw,
    quat_to_matrix_xyzw,
    scale_quat_rotation_xyzw,
    transform_point_inverse,
    validate_rotation_matrix,
)


class MappingMode(str, Enum):
    RELATIVE = "relative"
    ABSOLUTE = "absolute"


@dataclass
class ArmTarget:
    position: list[float]
    orientation_xyzw: list[float]

    @classmethod
    def from_pose_list(cls, pose) -> "ArmTarget":
        values = np.asarray(pose, dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError("pose must be [x, y, z, qx, qy, qz, qw]")
        return cls(values[:3].tolist(), normalize_quat_xyzw(values[3:]).tolist())

    def position_array(self) -> np.ndarray:
        arr = np.asarray(self.position, dtype=float)
        if arr.shape != (3,) or not np.isfinite(arr).all():
            raise ValueError("position must be 3 finite values")
        return arr

    def orientation_array(self) -> np.ndarray:
        return normalize_quat_xyzw(self.orientation_xyzw)

    def as_pose_list(self) -> list[float]:
        return [*self.position_array().tolist(), *self.orientation_array().tolist()]


@dataclass(frozen=True)
class _RelativeAnchor:
    robot: ArmTarget
    webxr: ArmTarget


@dataclass(frozen=True)
class ModeSwitchResult:
    accepted: bool
    reason: str = ""
    candidates: Mapping[str, ArmTarget] | None = None


class ArmMapper:
    def __init__(
        self,
        position_scale_xyz=(1.0, 1.0, 1.0),
        rotation_scale: float = 1.0,
        robot_from_vr_axes=None,
        enable_orientation: bool = True,
        mapping_mode: MappingMode | str = MappingMode.RELATIVE,
        position_scale: float | None = None,
    ):
        if position_scale is not None:
            position_scale_xyz = (position_scale, position_scale, position_scale)
        self.position_scale_xyz = np.asarray(position_scale_xyz, dtype=float)
        if self.position_scale_xyz.shape != (3,) or not np.isfinite(self.position_scale_xyz).all():
            raise ValueError("position_scale_xyz must contain 3 finite values")
        if np.any(self.position_scale_xyz < 0.0):
            raise ValueError("position_scale_xyz must be non-negative")
        self.rotation_scale = float(rotation_scale)
        if not np.isfinite(self.rotation_scale) or self.rotation_scale < 0.0 or self.rotation_scale > 1.0:
            raise ValueError("rotation_scale must be between 0 and 1")
        self.robot_from_vr_axes = validate_rotation_matrix(np.eye(3) if robot_from_vr_axes is None else robot_from_vr_axes)
        self._robot_from_vr_quat = matrix_to_quat_xyzw(self.robot_from_vr_axes)
        self.enable_orientation = bool(enable_orientation)
        self.mode = MappingMode(mapping_mode)
        self._relative: dict[str, _RelativeAnchor] = {}
        self._absolute: AbsoluteCalibrationResult | None = None
        self._last_orientation: dict[str, np.ndarray] = {}

    def engage(self, side: str, vr_pose: ArmTarget, robot_pose: ArmTarget) -> None:
        self._validate_side(side)
        self._validate_target(vr_pose)
        self._validate_target(robot_pose)
        self._relative[side] = _RelativeAnchor(robot=self._copy_target(robot_pose), webxr=self._copy_target(vr_pose))
        self._last_orientation[side] = robot_pose.orientation_array()

    def calibrate(self, side: str, robot_target: ArmTarget, webxr_target: ArmTarget) -> None:
        self.engage(side, webxr_target, robot_target)

    def recenter(self, side: str, vr_pose: ArmTarget, robot_pose: ArmTarget) -> None:
        if self.mode is MappingMode.ABSOLUTE:
            self.invalidate_absolute("recenter invalidated absolute calibration")
            raise RuntimeError("absolute mapping requires a new session calibration after recenter")
        self.engage(side, vr_pose, robot_pose)

    def reanchor_position_only(self, side: str, vr_pose: ArmTarget, robot_pose: ArmTarget) -> None:
        if self.mode is not MappingMode.RELATIVE:
            raise RuntimeError("position-only re-anchor requires relative mapping")
        self._validate_side(side)
        self._validate_target(vr_pose)
        self._validate_target(robot_pose)
        anchor = self._relative.get(side)
        if anchor is None:
            raise RuntimeError(f"{side} arm is not engaged")
        self._relative[side] = _RelativeAnchor(
            robot=ArmTarget(robot_pose.position_array().tolist(), anchor.robot.orientation_array().tolist()),
            webxr=ArmTarget(vr_pose.position_array().tolist(), anchor.webxr.orientation_array().tolist()),
        )

    def disengage(self, side: str | None = None) -> None:
        if side is None:
            self._relative.clear()
            self._last_orientation.clear()
            return
        self._validate_side(side)
        self._relative.pop(side, None)
        self._last_orientation.pop(side, None)

    def is_calibrated(self, side: str) -> bool:
        if self.mode is MappingMode.ABSOLUTE:
            return self._absolute is not None and self._absolute.valid and side in self._absolute.sides
        return side in self._relative

    def set_absolute_calibration(self, result: AbsoluteCalibrationResult) -> None:
        if not result.valid:
            raise ValueError("cannot install an invalid absolute calibration")
        self._absolute = result
        self._last_orientation.clear()

    def invalidate_absolute(self, reason: str) -> None:
        if self._absolute is not None:
            self._absolute.invalidate(reason)
        self._last_orientation.clear()

    def validate_absolute_context(
        self,
        xr_session_id: str,
        reference_space: str,
        reference_space_revision: int | None,
        hmd_valid: bool = True,
    ) -> None:
        if self._absolute is None:
            raise RuntimeError("absolute session is not calibrated")
        self._absolute.require_context(xr_session_id, reference_space, reference_space_revision, hmd_valid)

    def map_hand(
        self,
        side: str,
        current_webxr: ArmTarget,
        *,
        xr_session_id: str | None = None,
        reference_space: str | None = None,
        reference_space_revision: int | None = None,
        hmd_valid: bool = True,
    ) -> ArmTarget:
        self._validate_side(side)
        self._validate_target(current_webxr)
        if self.mode is MappingMode.RELATIVE:
            target = self._map_relative(side, current_webxr)
        else:
            if xr_session_id is None or reference_space is None:
                raise RuntimeError("absolute mapping requires WebXR session context")
            self.validate_absolute_context(xr_session_id, reference_space, reference_space_revision, hmd_valid)
            target = self._map_absolute(side, current_webxr)
        orientation = self._continuous_orientation(side, target.orientation_array())
        return ArmTarget(target.position_array().tolist(), orientation.tolist())

    def switch_mode(
        self,
        new_mode: MappingMode | str,
        teleop_state: str,
        current_hands: Mapping[str, ArmTarget] | None = None,
        robot_desired: Mapping[str, ArmTarget] | None = None,
        session_context: tuple[str, str, int | None] | None = None,
        max_position_jump_m: float = 0.05,
        max_rotation_jump_rad: float = 0.35,
        candidate_validator: Callable[[str, ArmTarget], str | None] | None = None,
    ) -> ModeSwitchResult:
        requested = MappingMode(new_mode)
        state = getattr(teleop_state, "value", teleop_state)
        if state not in {"IDLE", "ARMED", "PAUSED"}:
            return ModeSwitchResult(False, f"mapping mode cannot change while {state}")
        if requested is self.mode and requested is MappingMode.RELATIVE:
            return ModeSwitchResult(True, candidates={})
        candidates: dict[str, ArmTarget] = {}
        if requested is MappingMode.ABSOLUTE:
            if self._absolute is None or not self._absolute.valid:
                return ModeSwitchResult(False, "current WebXR session has no valid absolute calibration")
            if session_context is None:
                return ModeSwitchResult(False, "absolute mode switch requires WebXR session context")
            try:
                self.validate_absolute_context(*session_context)
                if current_hands is None or robot_desired is None:
                    raise RuntimeError("absolute mode switch requires current hand and robot poses")
                for side, hand_pose in current_hands.items():
                    if side not in self._absolute.sides:
                        continue
                    candidate = self._map_absolute(side, hand_pose)
                    desired = robot_desired[side]
                    position_jump = float(np.linalg.norm(candidate.position_array() - desired.position_array()))
                    rotation_jump = quat_angle_xyzw(candidate.orientation_array(), desired.orientation_array())
                    if position_jump > max_position_jump_m or rotation_jump > max_rotation_jump_rad:
                        return ModeSwitchResult(
                            False,
                            f"{side} candidate jump ({position_jump:.4f} m, {rotation_jump:.4f} rad) exceeds mode-switch limits",
                        )
                    candidates[side] = candidate
            except (KeyError, RuntimeError, ValueError) as exc:
                return ModeSwitchResult(False, str(exc))
            if candidate_validator is not None:
                for side, candidate in candidates.items():
                    reason = candidate_validator(side, candidate)
                    if reason:
                        return ModeSwitchResult(False, reason)
        self.mode = requested
        self._last_orientation.clear()
        self._relative.clear()
        return ModeSwitchResult(True, candidates=candidates)

    def _map_relative(self, side: str, current: ArmTarget) -> ArmTarget:
        if side not in self._relative:
            raise RuntimeError(f"{side} arm is not engaged")
        anchor = self._relative[side]
        delta_vr = current.position_array() - anchor.webxr.position_array()
        delta_robot = self.robot_from_vr_axes @ delta_vr
        position = anchor.robot.position_array() + self.position_scale_xyz * delta_robot
        orientation = anchor.robot.orientation_array()
        if self.enable_orientation:
            delta_vr_orientation = quat_multiply_xyzw(current.orientation_array(), quat_inverse_xyzw(anchor.webxr.orientation_array()))
            delta_robot_matrix = self.robot_from_vr_axes @ quat_to_matrix_xyzw(delta_vr_orientation) @ self.robot_from_vr_axes.T
            scaled_delta = scale_quat_rotation_xyzw(matrix_to_quat_xyzw(delta_robot_matrix), self.rotation_scale)
            orientation = quat_multiply_xyzw(scaled_delta, orientation)
        return ArmTarget(position.tolist(), orientation.tolist())

    def _map_absolute(self, side: str, current: ArmTarget) -> ArmTarget:
        if self._absolute is None or side not in self._absolute.sides:
            raise RuntimeError(f"{side} arm has no absolute calibration")
        side_calibration = self._absolute.sides[side]
        operator = self._absolute.operator_in_vr
        current_position_o = transform_point_inverse(
            operator.position_array(), operator.orientation_array(), current.position_array()
        )
        delta_operator = current_position_o - side_calibration.hand_in_operator.position_array()
        position = side_calibration.robot_anchor.position_array() + self.position_scale_xyz * (self.robot_from_vr_axes @ delta_operator)
        orientation = side_calibration.robot_anchor.orientation_array()
        if self.enable_orientation:
            current_orientation_o = quat_multiply_xyzw(quat_inverse_xyzw(operator.orientation_array()), current.orientation_array())
            neutral_orientation_o = side_calibration.hand_in_operator.orientation_array()
            delta_o = quat_multiply_xyzw(current_orientation_o, quat_inverse_xyzw(neutral_orientation_o))
            scaled_delta_o = scale_quat_rotation_xyzw(delta_o, self.rotation_scale)
            scaled_hand_o = quat_multiply_xyzw(scaled_delta_o, neutral_orientation_o)
            orientation = quat_multiply_xyzw(
                quat_multiply_xyzw(self._robot_from_vr_quat, scaled_hand_o),
                side_calibration.tool_alignment_xyzw,
            )
        return ArmTarget(position.tolist(), orientation.tolist())

    def _continuous_orientation(self, side: str, orientation) -> np.ndarray:
        current = normalize_quat_xyzw(orientation)
        previous = self._last_orientation.get(side)
        if previous is not None:
            current = align_quat_sign_xyzw(current, previous)
        self._last_orientation[side] = current
        return current

    @staticmethod
    def _validate_side(side: str) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")

    @staticmethod
    def _validate_target(target: ArmTarget) -> None:
        target.position_array()
        target.orientation_array()

    @staticmethod
    def _copy_target(target: ArmTarget) -> ArmTarget:
        return ArmTarget(target.position_array().tolist(), target.orientation_array().tolist())

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, Mapping

import numpy as np

from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import (
    mean_quaternion_xyzw,
    normalize_quat_xyzw,
    quat_angle_xyzw,
    quat_from_yaw_y_up,
    quat_inverse_xyzw,
    quat_multiply_xyzw,
    quat_to_matrix_xyzw,
    transform_point_inverse,
    validate_rotation_matrix,
    yaw_from_quat_y_up,
)


class CalibrationState(str, Enum):
    UNCALIBRATED = "UNCALIBRATED"
    COUNTDOWN = "COUNTDOWN"
    SAMPLING = "SAMPLING"
    VALID = "VALID"
    INVALID = "INVALID"


@dataclass(frozen=True)
class PoseSample:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    @classmethod
    def from_pose(cls, pose) -> "PoseSample":
        position = np.asarray(pose.position, dtype=float)
        orientation = normalize_quat_xyzw(pose.orientation_xyzw)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("pose position must contain 3 finite values")
        return cls(tuple(position.tolist()), tuple(orientation.tolist()))

    def position_array(self) -> np.ndarray:
        return np.asarray(self.position, dtype=float)

    def orientation_array(self) -> np.ndarray:
        return normalize_quat_xyzw(self.orientation_xyzw)


@dataclass(frozen=True)
class AbsoluteCalibrationSample:
    receive_time_ns: int
    xr_session_id: str
    reference_space: str
    reference_space_revision: int | None
    hmd: PoseSample
    hands: Mapping[str, PoseSample]
    robot_desired: Mapping[str, PoseSample]
    robot_current: Mapping[str, PoseSample] | None = None


@dataclass(frozen=True)
class AbsoluteCalibrationConfig:
    countdown_sec: float = 3.0
    sample_duration_sec: float = 1.5
    minimum_valid_samples: int = 60
    max_head_position_std_m: float = 0.010
    max_hand_position_std_m: float = 0.015
    max_head_yaw_std_rad: float = 0.050
    max_hand_rotation_std_rad: float = 0.090
    max_robot_position_std_m: float = 0.005
    max_robot_rotation_std_rad: float = 0.030
    max_robot_desired_current_error_m: float = 0.020
    require_reference_space_revision: bool = True


@dataclass(frozen=True)
class SideCalibration:
    hand_in_operator: PoseSample
    robot_anchor: PoseSample
    tool_alignment_xyzw: tuple[float, float, float, float]


@dataclass
class AbsoluteCalibrationResult:
    xr_session_id: str
    reference_space: str
    reference_space_revision: int
    start_time_ns: int
    end_time_ns: int
    enabled_sides: tuple[str, ...]
    operator_in_vr: PoseSample
    sides: dict[str, SideCalibration]
    quality: dict[str, float | int]
    valid: bool = True
    invalid_reason: str = ""

    def invalidate(self, reason: str) -> None:
        self.valid = False
        self.invalid_reason = str(reason)

    def require_context(
        self,
        xr_session_id: str,
        reference_space: str,
        reference_space_revision: int | None,
        hmd_valid: bool = True,
    ) -> None:
        if not self.valid:
            raise RuntimeError(f"absolute calibration is invalid: {self.invalid_reason}")
        if not hmd_valid:
            self.invalidate("HMD tracking invalid")
        elif xr_session_id != self.xr_session_id:
            self.invalidate("WebXR session changed")
        elif reference_space != self.reference_space:
            self.invalidate("WebXR reference space changed")
        elif reference_space_revision is None:
            self.invalidate("reference_space_revision is required for absolute mapping")
        elif int(reference_space_revision) != self.reference_space_revision:
            self.invalidate("WebXR reference space revision changed")
        if not self.valid:
            raise RuntimeError(f"absolute calibration is invalid: {self.invalid_reason}")


class CalibrationError(ValueError):
    pass


def _robust_position_mean(poses: Iterable[PoseSample]) -> tuple[np.ndarray, float]:
    values = np.asarray([pose.position_array() for pose in poses], dtype=float)
    if len(values) == 0:
        raise CalibrationError("no valid position samples")
    median = np.median(values, axis=0)
    distances = np.linalg.norm(values - median, axis=1)
    median_distance = float(np.median(distances))
    if median_distance > 0.0:
        keep = distances <= median_distance + 3.0 * float(np.median(np.abs(distances - median_distance)))
        if np.any(keep):
            values = values[keep]
    mean = np.mean(values, axis=0)
    std = float(np.sqrt(np.mean(np.sum((values - mean) ** 2, axis=1))))
    return mean, std


def _rotation_mean_std(poses: Iterable[PoseSample]) -> tuple[np.ndarray, float]:
    quaternions = [pose.orientation_array() for pose in poses]
    if not quaternions:
        raise CalibrationError("no valid rotation samples")
    mean = mean_quaternion_xyzw(quaternions)
    std = float(np.sqrt(np.mean([quat_angle_xyzw(mean, value) ** 2 for value in quaternions])))
    return mean, std


def _yaw_mean_std(poses: Iterable[PoseSample]) -> tuple[float, float]:
    yaws = np.asarray([yaw_from_quat_y_up(pose.orientation_array()) for pose in poses], dtype=float)
    if len(yaws) == 0:
        raise CalibrationError("no valid HMD yaw samples")
    mean = float(np.arctan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws))))
    errors = np.arctan2(np.sin(yaws - mean), np.cos(yaws - mean))
    return mean, float(np.sqrt(np.mean(errors**2)))


def build_absolute_calibration(
    samples: Iterable[AbsoluteCalibrationSample],
    enabled_sides: Iterable[str],
    robot_from_operator_axes,
    config: AbsoluteCalibrationConfig | None = None,
) -> AbsoluteCalibrationResult:
    cfg = config or AbsoluteCalibrationConfig()
    sample_list = list(samples)
    sides = tuple(enabled_sides)
    if not sides or any(side not in {"left", "right"} for side in sides):
        raise CalibrationError("enabled_sides must contain left and/or right")
    if len(sample_list) < cfg.minimum_valid_samples:
        raise CalibrationError(f"sample count {len(sample_list)} is below minimum {cfg.minimum_valid_samples}")
    first = sample_list[0]
    if not first.xr_session_id:
        raise CalibrationError("xr_session_id is required")
    if first.reference_space not in {"local", "local-floor"}:
        raise CalibrationError(f"unsupported reference space: {first.reference_space}")
    if cfg.require_reference_space_revision and first.reference_space_revision is None:
        raise CalibrationError("reference_space_revision is required")
    revision = int(first.reference_space_revision or 0)
    for index, sample in enumerate(sample_list):
        if (
            sample.xr_session_id != first.xr_session_id
            or sample.reference_space != first.reference_space
            or sample.reference_space_revision != first.reference_space_revision
        ):
            raise CalibrationError(f"session/reference space changed at sample {index}")
        missing = [side for side in sides if side not in sample.hands or side not in sample.robot_desired]
        if missing:
            raise CalibrationError(f"sample {index} missing enabled side: {', '.join(missing)}")

    quality: dict[str, float | int] = {"sample_count": len(sample_list)}
    head_position, head_position_std = _robust_position_mean(sample.hmd for sample in sample_list)
    head_yaw, head_yaw_std = _yaw_mean_std(sample.hmd for sample in sample_list)
    quality["head_position_std_m"] = head_position_std
    quality["head_yaw_std_rad"] = head_yaw_std
    if head_position_std > cfg.max_head_position_std_m:
        raise CalibrationError(f"head position std {head_position_std:.6f} m exceeds {cfg.max_head_position_std_m:.6f} m")
    if head_yaw_std > cfg.max_head_yaw_std_rad:
        raise CalibrationError(f"head yaw std {head_yaw_std:.6f} rad exceeds {cfg.max_head_yaw_std_rad:.6f} rad")

    operator_position = head_position.copy()
    if first.reference_space == "local-floor":
        operator_position[1] = 0.0
    operator_orientation = quat_from_yaw_y_up(head_yaw)
    robot_from_operator = validate_rotation_matrix(robot_from_operator_axes)
    robot_from_operator_quat = _matrix_quat(robot_from_operator)
    result_sides: dict[str, SideCalibration] = {}

    for side in sides:
        hand_poses = [sample.hands[side] for sample in sample_list]
        robot_poses = [sample.robot_desired[side] for sample in sample_list]
        hand_position, hand_position_std = _robust_position_mean(hand_poses)
        hand_orientation, hand_rotation_std = _rotation_mean_std(hand_poses)
        robot_position, robot_position_std = _robust_position_mean(robot_poses)
        robot_orientation, robot_rotation_std = _rotation_mean_std(robot_poses)
        quality[f"{side}_hand_position_std_m"] = hand_position_std
        quality[f"{side}_hand_rotation_std_rad"] = hand_rotation_std
        quality[f"{side}_robot_position_std_m"] = robot_position_std
        quality[f"{side}_robot_rotation_std_rad"] = robot_rotation_std
        limits = (
            (hand_position_std, cfg.max_hand_position_std_m, "hand position std", "m"),
            (hand_rotation_std, cfg.max_hand_rotation_std_rad, "hand rotation std", "rad"),
            (robot_position_std, cfg.max_robot_position_std_m, "robot position std", "m"),
            (robot_rotation_std, cfg.max_robot_rotation_std_rad, "robot rotation std", "rad"),
        )
        for value, limit, label, unit in limits:
            if value > limit:
                raise CalibrationError(f"{side} {label} {value:.6f} {unit} exceeds {limit:.6f} {unit}")
        current_errors = []
        for sample in sample_list:
            if sample.robot_current is not None and side in sample.robot_current:
                current_errors.append(
                    float(np.linalg.norm(sample.robot_desired[side].position_array() - sample.robot_current[side].position_array()))
                )
        max_current_error = max(current_errors, default=0.0)
        quality[f"{side}_robot_desired_current_error_m"] = max_current_error
        if max_current_error > cfg.max_robot_desired_current_error_m:
            raise CalibrationError(
                f"{side} robot desired/current error {max_current_error:.6f} m exceeds {cfg.max_robot_desired_current_error_m:.6f} m"
            )

        hand_in_operator_position = transform_point_inverse(operator_position, operator_orientation, hand_position)
        hand_in_operator_orientation = quat_multiply_xyzw(quat_inverse_xyzw(operator_orientation), hand_orientation)
        robot_from_hand_at_neutral = quat_multiply_xyzw(robot_from_operator_quat, hand_in_operator_orientation)
        tool_alignment = quat_multiply_xyzw(quat_inverse_xyzw(robot_from_hand_at_neutral), robot_orientation)
        result_sides[side] = SideCalibration(
            hand_in_operator=PoseSample(tuple(hand_in_operator_position.tolist()), tuple(hand_in_operator_orientation.tolist())),
            robot_anchor=PoseSample(tuple(robot_position.tolist()), tuple(robot_orientation.tolist())),
            tool_alignment_xyzw=tuple(tool_alignment.tolist()),
        )

    return AbsoluteCalibrationResult(
        xr_session_id=first.xr_session_id,
        reference_space=first.reference_space,
        reference_space_revision=revision,
        start_time_ns=min(sample.receive_time_ns for sample in sample_list),
        end_time_ns=max(sample.receive_time_ns for sample in sample_list),
        enabled_sides=sides,
        operator_in_vr=PoseSample(tuple(operator_position.tolist()), tuple(operator_orientation.tolist())),
        sides=result_sides,
        quality=quality,
    )


def _matrix_quat(matrix) -> np.ndarray:
    from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import matrix_to_quat_xyzw

    return matrix_to_quat_xyzw(matrix)


class AbsoluteSessionCalibrator:
    def __init__(self, config: AbsoluteCalibrationConfig | None = None):
        self.config = config or AbsoluteCalibrationConfig()
        self.state = CalibrationState.UNCALIBRATED
        self.failure_reason = ""
        self.result: AbsoluteCalibrationResult | None = None
        self._samples: list[AbsoluteCalibrationSample] = []
        self._start_ns = 0
        self._enabled_sides: tuple[str, ...] = ()
        self._robot_from_operator = np.eye(3)
        self._sample_context: tuple[str, str, int | None] | None = None

    def start(self, now_ns: int, enabled_sides: Iterable[str], robot_from_operator_axes) -> None:
        self.cancel("calibration restarted", uncalibrated=True)
        self._start_ns = int(now_ns)
        self._enabled_sides = tuple(enabled_sides)
        self._robot_from_operator = validate_rotation_matrix(robot_from_operator_axes)
        self._sample_context = None
        self.state = CalibrationState.COUNTDOWN if self.config.countdown_sec > 0.0 else CalibrationState.SAMPLING

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def progress(self, now_ns: int) -> dict[str, float | int | str]:
        elapsed = max(0.0, (int(now_ns) - self._start_ns) / 1e9)
        countdown_remaining = max(0.0, self.config.countdown_sec - elapsed)
        sampling_elapsed = max(0.0, elapsed - self.config.countdown_sec)
        sampling_progress = min(1.0, sampling_elapsed / self.config.sample_duration_sec) if self.config.sample_duration_sec > 0 else 1.0
        return {
            "state": self.state.value,
            "countdown_remaining_sec": countdown_remaining,
            "sampling_progress": sampling_progress,
            "sample_count": self.sample_count,
            "minimum_valid_samples": self.config.minimum_valid_samples,
        }

    def add_sample(self, sample: AbsoluteCalibrationSample) -> CalibrationState:
        if self.state not in {CalibrationState.COUNTDOWN, CalibrationState.SAMPLING}:
            raise RuntimeError("calibration is not collecting samples")
        elapsed = (int(sample.receive_time_ns) - self._start_ns) / 1e9
        if elapsed < 0.0:
            return self.state
        if self.state is CalibrationState.COUNTDOWN:
            if elapsed < self.config.countdown_sec:
                return self.state
            self.state = CalibrationState.SAMPLING
        context = (sample.xr_session_id, sample.reference_space, sample.reference_space_revision)
        if self._sample_context is None:
            self._sample_context = context
        elif context != self._sample_context:
            return self._fail("session/reference space changed during calibration")
        self._samples.append(sample)
        if elapsed >= self.config.countdown_sec + self.config.sample_duration_sec:
            self.finish()
        return self.state

    def finish(self) -> AbsoluteCalibrationResult:
        if self.state is not CalibrationState.SAMPLING:
            raise RuntimeError("calibration is not sampling")
        try:
            self.result = build_absolute_calibration(
                self._samples,
                self._enabled_sides,
                self._robot_from_operator,
                self.config,
            )
        except (CalibrationError, ValueError) as exc:
            self._fail(str(exc))
            raise CalibrationError(str(exc)) from exc
        self.state = CalibrationState.VALID
        self.failure_reason = ""
        return self.result

    def cancel(self, reason: str = "calibration cancelled", uncalibrated: bool = False) -> None:
        if self.result is not None:
            self.result.invalidate(reason)
        self.result = None
        self._samples.clear()
        self._sample_context = None
        self.failure_reason = "" if uncalibrated else reason
        self.state = CalibrationState.UNCALIBRATED if uncalibrated else CalibrationState.INVALID

    def invalidate(self, reason: str) -> None:
        self.cancel(reason)

    def _fail(self, reason: str) -> CalibrationState:
        self.cancel(reason)
        return self.state

from __future__ import annotations

import json
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from threading import Lock
from time import monotonic, monotonic_ns, sleep
from typing import Any

import numpy as np
import yaml

from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import (
    AbsoluteCalibrationConfig,
    AbsoluteSessionCalibrator,
    CalibrationState,
)
from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmMapper, ArmTarget, MappingMode
from stardust_wuji_quest3_pc_retargeting.arm_control.astribot_adapter import AstribotAdapter
from stardust_wuji_quest3_pc_retargeting.conversion.webxr_to_mp21 import WebXRToMP21Converter
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import (
    mean_quaternion_xyzw,
    quat_angle_xyzw,
    quat_slerp_xyzw,
    yaw_from_quat_y_up,
)
from stardust_wuji_quest3_pc_retargeting.hand_control.retarget_pipeline import RetargetPipeline
from stardust_wuji_quest3_pc_retargeting.protocol.validation import validate_tracking_frame
from stardust_wuji_quest3_pc_retargeting.runtime.arm_control_loop import ArmControlLoop, ArmFrameProcessor, LoopState, PauseControl
from stardust_wuji_quest3_pc_retargeting.runtime.control_commands import CommandResult, ControlCommand, ControlCommandQueue
from stardust_wuji_quest3_pc_retargeting.runtime.latest_tracking import LatestTrackingBuffer
from stardust_wuji_quest3_pc_retargeting.safety.arm_safety_filter import ArmCommand, ArmSafetyFilter
from stardust_wuji_quest3_pc_retargeting.safety.hand_safety_filter import HandCommand, HandSafetyFilter
from stardust_wuji_quest3_pc_retargeting.safety.state_machine import TeleopState, TeleopStateMachine


@dataclass
class SupervisorOutput:
    state: str
    hands: dict[str, HandCommand]
    arms: dict[str, ArmCommand]
    seq: int


@dataclass(frozen=True)
class SupervisorStatus:
    teleop_state: str
    mapping_mode: str
    arm_sides: tuple[str, ...]
    dry_run: bool
    webxr_active: bool
    xr_session_id: str
    reference_space: str
    reference_space_revision: int | None
    hmd_tracking: bool
    hand_tracking: dict[str, bool]
    arm_wrist_tracking: dict[str, bool]
    robot_connected: bool
    control_rights: bool
    calibration_state: str
    calibration_failure_reason: str
    calibration_progress: dict[str, float | int | str]
    calibration_quality: dict[str, float | int]
    loop_state: str
    last_error: str
    last_command_message: str
    sent_cycles: int
    missed_deadlines: int
    sdk_call_p50_ms: float
    sdk_call_p95_ms: float
    sdk_call_max_ms: float
    use_wbc: bool
    add_default_torso: bool
    locked_groups: tuple[str, ...]
    workspace_xyz_min: dict[str, tuple[float, float, float]]
    workspace_xyz_max: dict[str, tuple[float, float, float]]
    position_scale_xyz: tuple[float, float, float]
    max_linear_speed_mps: float
    max_input_position_jump_m: float
    start_max_position_jump_m: float
    position_alpha: float
    orientation_alpha: float
    position_lead_sec: float
    max_position_lead_m: float
    enable_orientation: bool
    rotation_scale: float
    max_angular_speed_rad_s: float
    hand_reacquire_timeout_sec: float
    hand_reacquire_stable_frames: int
    hand_reacquire_invalid_grace_frames: int
    hand_reacquire_state: str
    hand_reacquire_sides: tuple[str, ...]
    hand_reacquire_remaining_sec: float
    absolute_orientation_reacquire: bool
    orientation_reacquire_speed_rad_s: float
    reacquire_position_errors_m: dict[str, float]
    reacquire_orientation_errors_rad: dict[str, float]
    reacquire_candidate_positions_m: dict[str, list[float]]
    reacquire_workspace_violations: dict[str, list[str]]
    last_arm_filter_rejections: dict[str, str]
    hand_reacquire_trigger_reason: str
    hand_tracking_loss_events: int
    hand_tracking_catchup_interruptions: int
    hand_tracking_recovery_completions: int
    absolute_pose_reacquire: bool
    absolute_reacquire_linear_speed_mps: float
    pose_reacquire_linear_accel_mps2: float
    pose_reacquire_angular_accel_rad_s2: float
    fixed_anchor_mode: bool
    init_recovery_enabled: bool
    init_recovery_duration_sec: float
    engage_state: str
    engage_sample_count: int
    engage_stable_frames: int
    engage_operator_position_m: tuple[float, float, float]
    engage_operator_yaw_rad: float
    engage_hold_remaining_sec: float
    engage_soft_start_remaining_sec: float


class ControlPCSupervisor:
    def __init__(
        self,
        arm_config: dict[str, Any],
        arm: str = "both",
        mapping_mode: MappingMode | str | None = None,
        enable_real_arm: bool = False,
        absolute_calibration_report: str | Path | None = None,
        adapter: AstribotAdapter | None = None,
        sdk_root: str | Path = "/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master",
        high_control_rights: bool = False,
        allow_orientation_control: bool = False,
        allow_real_absolute: bool = False,
    ) -> None:
        if arm not in {"left", "right", "both"}:
            raise ValueError("arm must be left, right, or both")
        self.arm_config = arm_config
        self.fixed_anchor_mode = bool(arm_config.get("safety", {}).get("fixed_anchor_mode", False))
        self.init_recovery_config = dict(arm_config.get("init_recovery", {}))
        self.init_recovery_enabled = bool(
            arm_config.get("safety", {}).get("init_recovery_enabled", False)
        )
        self.enabled_sides = ("left", "right") if arm == "both" else (arm,)
        mapping = arm_config.get("mapping", {})
        selected_mode = MappingMode(mapping_mode or mapping.get("mode", "relative"))
        if enable_real_arm and arm not in {"left", "both"}:
            raise RuntimeError("M8 real arm is limited to left or both arms")
        if enable_real_arm and selected_mode is MappingMode.ABSOLUTE and not allow_real_absolute:
            raise RuntimeError("M8 real absolute mode requires explicit full-absolute authorization")
        if enable_real_arm and bool(mapping.get("enable_orientation", True)) and not allow_orientation_control:
            raise RuntimeError("M8 real arm requires orientation control disabled")
        whole_body = arm_config.get("whole_body", {})
        if enable_real_arm and (
            bool(arm_config.get("use_wbc", False))
            or bool(arm_config.get("add_default_torso", False))
            or bool(whole_body.get("enabled", False))
            or any(bool(whole_body.get(key, False)) for key in ("allow_torso", "allow_chassis", "allow_head"))
        ):
            raise RuntimeError("M8 real arm requires WBC, torso, chassis, and head control to remain disabled")
        if adapter is not None and adapter.enable_real != bool(enable_real_arm):
            raise RuntimeError("injected adapter real/dry-run mode does not match supervisor mode")
        self.adapter = adapter or AstribotAdapter(
            freq_hz=float(arm_config.get("control_rate_hz", 100.0)),
            enable_real=bool(enable_real_arm),
            control_way=str(arm_config.get("control_way", "filter")),
            use_wbc=bool(arm_config.get("use_wbc", False)),
            add_default_torso=bool(arm_config.get("add_default_torso", False)),
            allow_whole_body=bool(whole_body.get("enabled", False)),
            sdk_root=sdk_root,
            high_control_rights=bool(high_control_rights),
        )
        if not self.adapter.enable_real:
            self._seed_dryrun_robot_poses()
        self.mapper = ArmMapper(
            position_scale_xyz=mapping.get("position_scale_xyz", [1.0, 1.0, 1.0]),
            rotation_scale=float(mapping.get("rotation_scale", 1.0)),
            robot_from_vr_axes=mapping.get("robot_from_vr_axes", np.eye(3)),
            enable_orientation=bool(mapping.get("enable_orientation", True)),
            mapping_mode=selected_mode,
        )
        safety = arm_config.get("safety", {})
        filtering = arm_config.get("filter", {})
        arm_entries = arm_config.get("arms", {})
        self.arm_filters = {
            side: ArmSafetyFilter(
                xyz_min=arm_entries.get(side, {}).get("workspace_xyz_min", [-2, -2, -2]),
                xyz_max=arm_entries.get(side, {}).get("workspace_xyz_max", [2, 2, 2]),
                max_linear_speed_mps=float(safety.get("max_linear_speed_mps", 0.1)),
                max_angular_speed_rad_s=float(safety.get("max_angular_speed_rad_s", 0.5)),
                max_input_position_jump_m=float(safety.get("max_input_position_jump_m", 0.1)),
                max_input_rotation_jump_rad=float(safety.get("max_input_rotation_jump_rad", 0.8)),
                minimum_dt_sec=float(safety.get("minimum_dt_sec", 0.002)),
                maximum_dt_sec=float(safety.get("maximum_dt_sec", 0.05)),
                position_alpha=float(filtering.get("position_alpha", 1.0)),
                orientation_alpha=float(filtering.get("orientation_alpha", 1.0)),
                position_lead_sec=float(filtering.get("position_lead_sec", 0.0)),
                max_position_lead_m=float(filtering.get("max_position_lead_m", 0.0)),
            )
            for side in self.enabled_sides
        }
        calibration_fields = AbsoluteCalibrationConfig.__dataclass_fields__
        calibration_values = {
            key: value
            for key, value in arm_config.get("absolute_session_calibration", {}).items()
            if key in calibration_fields
        }
        self.calibrator = AbsoluteSessionCalibrator(AbsoluteCalibrationConfig(**calibration_values))
        self.frame_processor = ArmFrameProcessor(
            self.mapper,
            self.arm_filters,
            self.enabled_sides,
            adapter=self.adapter,
            calibrator=self.calibrator,
            hand_reacquire_timeout_sec=float(safety.get("hand_reacquire_timeout_sec", 0.0)),
            hand_reacquire_stable_frames=int(safety.get("hand_reacquire_stable_frames", 3)),
            hand_reacquire_invalid_grace_frames=int(
                safety.get("hand_reacquire_invalid_grace_frames", 2)
            ),
            absolute_orientation_reacquire=bool(safety.get("absolute_orientation_reacquire", False)),
            orientation_reacquire_speed_rad_s=float(safety.get("orientation_reacquire_speed_rad_s", 0.5)),
            orientation_reacquire_direct_error_rad=float(
                safety.get("orientation_reacquire_direct_error_rad", 0.15)
            ),
            orientation_reacquire_complete_error_rad=float(
                safety.get("orientation_reacquire_complete_error_rad", 0.087)
            ),
            orientation_reacquire_max_error_rad=float(
                safety.get("orientation_reacquire_max_error_rad", 1.57)
            ),
            orientation_reacquire_complete_frames=int(
                safety.get("orientation_reacquire_complete_frames", 5)
            ),
            absolute_pose_reacquire=bool(safety.get("absolute_pose_reacquire", False)),
            absolute_reacquire_linear_speed_mps=float(
                safety.get("absolute_reacquire_linear_speed_mps", 0.10)
            ),
            absolute_reacquire_direct_position_error_m=float(
                safety.get("absolute_reacquire_direct_position_error_m", 0.02)
            ),
            absolute_reacquire_complete_position_error_m=float(
                safety.get("absolute_reacquire_complete_position_error_m", 0.01)
            ),
            absolute_reacquire_max_position_error_m=float(
                safety.get("absolute_reacquire_max_position_error_m", 0.20)
            ),
            fixed_anchor_pose_reacquire=bool(safety.get("fixed_anchor_pose_reacquire", False)),
            pose_reacquire_linear_accel_mps2=float(
                safety.get("pose_reacquire_linear_accel_mps2", 0.30)
            ),
            pose_reacquire_angular_accel_rad_s2=float(
                safety.get("pose_reacquire_angular_accel_rad_s2", 1.50)
            ),
        )
        self.tracking_buffer = LatestTrackingBuffer()
        self.commands = ControlCommandQueue()
        self.state_machine = TeleopStateMachine()
        self._status_lock = Lock()
        self._latest_frame = None
        self._last_error = ""
        self._last_command_message = ""
        self._absolute_auto_start_after_calibration = False
        self.engage_stable_frames = int(safety.get("engage_stable_frames", 12))
        self.engage_timeout_sec = float(safety.get("engage_timeout_sec", 5.0))
        self.engage_hmd_position_stability_m = float(
            safety.get("engage_hmd_position_stability_m", 0.010)
        )
        self.engage_hmd_yaw_stability_rad = float(
            safety.get("engage_hmd_yaw_stability_rad", 0.030)
        )
        self.engage_wrist_position_stability_m = float(
            safety.get("engage_wrist_position_stability_m", 0.005)
        )
        self.engage_wrist_rotation_stability_rad = float(
            safety.get("engage_wrist_rotation_stability_rad", 0.050)
        )
        self.engage_hold_duration_sec = float(
            safety.get("engage_hold_duration_sec", 0.25)
        )
        self.engage_soft_start_duration_sec = float(
            safety.get("engage_soft_start_duration_sec", 0.50)
        )
        if self.engage_stable_frames < 1:
            raise ValueError("engage stable frames must be positive")
        if not 0.0 < self.engage_timeout_sec <= 30.0:
            raise ValueError("engage timeout must be in (0, 30] seconds")
        if any(
            value <= 0.0
            for value in (
                self.engage_hmd_position_stability_m,
                self.engage_hmd_yaw_stability_rad,
                self.engage_wrist_position_stability_m,
                self.engage_wrist_rotation_stability_rad,
            )
        ):
            raise ValueError("engage stability thresholds must be positive")
        if self.engage_hold_duration_sec < 0.0 or self.engage_soft_start_duration_sec < 0.0:
            raise ValueError("engage hold and soft-start durations must be non-negative")
        self._engage_state = "IDLE"
        self._engage_samples = deque(maxlen=self.engage_stable_frames)
        self._engage_started_ns: int | None = None
        self._engage_soft_start_started_ns: int | None = None
        self._engage_robot_anchors: dict[str, ArmTarget] = {}
        self._engage_context: tuple[str, str, int | None] | None = None
        self._engage_last_sample_key: tuple[str, int] | None = None
        self._engage_operator_position = np.zeros(3, dtype=float)
        self._calibration_report = None if absolute_calibration_report is None else Path(absolute_calibration_report).expanduser()
        self.loop = ArmControlLoop(
            self.adapter,
            self.tracking_buffer,
            self._process_control_frame,
            control_rate_hz=float(arm_config.get("control_rate_hz", 100.0)),
            fresh_timeout_sec=float(safety.get("fresh_timeout_sec", 0.05)),
            disable_timeout_sec=float(safety.get("disable_timeout_sec", 0.10)),
            sdk_block_fault_sec=float(safety.get("sdk_block_fault_sec", 0.020)),
            consecutive_deadline_fault_count=int(safety.get("consecutive_deadline_fault_count", 3)),
            command_pump=self._pump_commands,
        )

    def start(self) -> None:
        self.loop.start()

    def wait_until_adapter_ready(self, timeout_sec: float = 20.0) -> None:
        deadline = monotonic() + float(timeout_sec)
        while monotonic() < deadline:
            if self.adapter.initialized:
                return
            if self.loop.state is LoopState.FAULT:
                raise RuntimeError(self.loop.fault_reason or "real arm initialization failed")
            sleep(0.01)
        raise RuntimeError("timed out waiting for Astribot adapter initialization")

    def close(self) -> None:
        self.loop.stop()

    def ingest_payload(self, payload: dict[str, Any], receive_time_ns: int | None = None):
        frame = validate_tracking_frame(payload)
        received = monotonic_ns() if receive_time_ns is None else int(receive_time_ns)
        with self._status_lock:
            previous = self._latest_frame
            self._latest_frame = frame
        if previous is not None and self.calibrator.state is CalibrationState.VALID:
            changed = (
                not frame.session.active
                or not frame.hmd.valid
                or frame.xr_session_id != previous.xr_session_id
                or frame.session.reference_space != previous.session.reference_space
                or frame.session.reference_space_revision != previous.session.reference_space_revision
            )
            if changed:
                self.submit_command("invalidate-calibration", "WebXR session/reference-space changed")
        return self.tracking_buffer.publish(frame, received)

    def submit_command(self, name: str, argument: str | None = None) -> ControlCommand:
        return self.commands.submit(name, argument)

    def wait_command(self, command: ControlCommand, timeout: float | None = 2.0) -> CommandResult | None:
        return self.commands.wait(command, timeout)

    def execute_command(self, name: str, argument: str | None = None, timeout: float = 2.0) -> CommandResult:
        command = self.submit_command(name, argument)
        result = self.wait_command(command, timeout)
        if result is None:
            raise TimeoutError(f"command {name} was not processed within {timeout:.1f} s")
        return result

    def status_snapshot(self, now_ns: int | None = None) -> SupervisorStatus:
        now = monotonic_ns() if now_ns is None else int(now_ns)
        with self._status_lock:
            frame = self._latest_frame
            last_error = self._last_error or (self.loop.fault_reason if self.loop.state is LoopState.FAULT else "")
            last_command = self._last_command_message
        calibration_progress = self.calibrator.progress(now)
        calibration_quality = dict(self.calibrator.result.quality) if self.calibrator.result is not None else {}
        hand_reacquire = self.frame_processor.tracking_reacquire_status(now)
        teleop_state = self.state_machine.state.value
        if self.loop.state is LoopState.PAUSED and teleop_state == TeleopState.RUNNING.value:
            teleop_state = TeleopState.PAUSED.value
        if hand_reacquire["state"] != "IDLE" and teleop_state == TeleopState.RUNNING.value:
            teleop_state = TeleopState.PAUSED.value
        if self.loop.state is LoopState.FAULT:
            teleop_state = TeleopState.FAULT.value
        sdk_ms = np.asarray(self.loop.stats.sdk_call_time_ns, dtype=float) / 1e6
        engage_hold_remaining = 0.0
        engage_soft_start_remaining = 0.0
        if self._engage_state == "SOFT_START" and self._engage_soft_start_started_ns is not None:
            elapsed = max(0.0, (now - self._engage_soft_start_started_ns) / 1e9)
            engage_hold_remaining = max(0.0, self.engage_hold_duration_sec - elapsed)
            engage_soft_start_remaining = max(
                0.0,
                self.engage_hold_duration_sec + self.engage_soft_start_duration_sec - elapsed,
            )
        return SupervisorStatus(
            teleop_state=teleop_state,
            mapping_mode=self.mapper.mode.value,
            arm_sides=self.enabled_sides,
            dry_run=not self.adapter.enable_real,
            webxr_active=bool(frame and frame.session.active),
            xr_session_id="" if frame is None else frame.xr_session_id,
            reference_space="" if frame is None else frame.session.reference_space,
            reference_space_revision=None if frame is None else frame.session.reference_space_revision,
            hmd_tracking=bool(frame and frame.hmd.valid),
            hand_tracking={side: bool(frame and frame.hands[side].valid) for side in self.enabled_sides},
            arm_wrist_tracking={
                side: bool(frame and frame.arm_wrists[side].valid) for side in self.enabled_sides
            },
            robot_connected=self.adapter.initialized,
            control_rights=self.adapter.control_rights,
            calibration_state=self.calibrator.state.value,
            calibration_failure_reason=self.calibrator.failure_reason,
            calibration_progress=calibration_progress,
            calibration_quality=calibration_quality,
            loop_state=self.loop.state.value,
            last_error=last_error,
            last_command_message=last_command,
            sent_cycles=self.loop.stats.sent_cycles,
            missed_deadlines=self.loop.stats.missed_deadlines,
            sdk_call_p50_ms=0.0 if sdk_ms.size == 0 else float(np.percentile(sdk_ms, 50)),
            sdk_call_p95_ms=0.0 if sdk_ms.size == 0 else float(np.percentile(sdk_ms, 95)),
            sdk_call_max_ms=0.0 if sdk_ms.size == 0 else float(np.max(sdk_ms)),
            use_wbc=bool(self.adapter.use_wbc),
            add_default_torso=bool(self.adapter.add_default_torso),
            locked_groups=("torso", "chassis", "head") if not self.adapter.allow_whole_body else (),
            workspace_xyz_min={
                side: tuple(float(value) for value in self.arm_filters[side].xyz_min)
                for side in self.enabled_sides
            },
            workspace_xyz_max={
                side: tuple(float(value) for value in self.arm_filters[side].xyz_max)
                for side in self.enabled_sides
            },
            position_scale_xyz=tuple(float(value) for value in self.mapper.position_scale_xyz),
            max_linear_speed_mps=float(next(iter(self.arm_filters.values())).max_linear_speed_mps),
            max_input_position_jump_m=float(next(iter(self.arm_filters.values())).max_input_position_jump_m),
            start_max_position_jump_m=float(self.arm_config.get("safety", {}).get("mode_switch_max_position_jump_m", 0.05)),
            position_alpha=float(self.arm_config.get("filter", {}).get("position_alpha", 1.0)),
            orientation_alpha=float(self.arm_config.get("filter", {}).get("orientation_alpha", 1.0)),
            position_lead_sec=float(self.arm_config.get("filter", {}).get("position_lead_sec", 0.0)),
            max_position_lead_m=float(self.arm_config.get("filter", {}).get("max_position_lead_m", 0.0)),
            enable_orientation=bool(self.mapper.enable_orientation),
            rotation_scale=float(self.mapper.rotation_scale),
            max_angular_speed_rad_s=float(next(iter(self.arm_filters.values())).max_angular_speed_rad_s),
            hand_reacquire_timeout_sec=float(self.frame_processor.hand_reacquire_timeout_sec),
            hand_reacquire_stable_frames=int(self.frame_processor.hand_reacquire_stable_frames),
            hand_reacquire_invalid_grace_frames=int(
                self.frame_processor.hand_reacquire_invalid_grace_frames
            ),
            hand_reacquire_state=str(hand_reacquire["state"]),
            hand_reacquire_sides=tuple(hand_reacquire["sides"]),
            hand_reacquire_remaining_sec=float(hand_reacquire["remaining_sec"]),
            absolute_orientation_reacquire=bool(self.frame_processor.absolute_orientation_reacquire),
            orientation_reacquire_speed_rad_s=float(
                self.frame_processor.orientation_reacquire_speed_rad_s
            ),
            reacquire_position_errors_m=dict(hand_reacquire["position_errors_m"]),
            reacquire_orientation_errors_rad=dict(hand_reacquire["orientation_errors_rad"]),
            reacquire_candidate_positions_m=dict(hand_reacquire["candidate_positions_m"]),
            reacquire_workspace_violations=dict(hand_reacquire["workspace_violations"]),
            last_arm_filter_rejections=dict(hand_reacquire["filter_rejections"]),
            hand_reacquire_trigger_reason=str(hand_reacquire["trigger_reason"]),
            hand_tracking_loss_events=int(hand_reacquire["loss_events"]),
            hand_tracking_catchup_interruptions=int(hand_reacquire["catchup_interruptions"]),
            hand_tracking_recovery_completions=int(hand_reacquire["recovery_completions"]),
            absolute_pose_reacquire=bool(self.frame_processor.absolute_pose_reacquire),
            absolute_reacquire_linear_speed_mps=float(
                self.frame_processor.absolute_reacquire_linear_speed_mps
            ),
            pose_reacquire_linear_accel_mps2=float(
                self.frame_processor.pose_reacquire_linear_accel_mps2
            ),
            pose_reacquire_angular_accel_rad_s2=float(
                self.frame_processor.pose_reacquire_angular_accel_rad_s2
            ),
            fixed_anchor_mode=self.fixed_anchor_mode,
            init_recovery_enabled=self.init_recovery_enabled,
            init_recovery_duration_sec=float(
                self.init_recovery_config.get("duration_sec", 4.0)
            ),
            engage_state=self._engage_state,
            engage_sample_count=len(self._engage_samples),
            engage_stable_frames=self.engage_stable_frames,
            engage_operator_position_m=tuple(
                float(value) for value in self._engage_operator_position
            ),
            engage_operator_yaw_rad=self.mapper.relative_operator_yaw_rad,
            engage_hold_remaining_sec=engage_hold_remaining,
            engage_soft_start_remaining_sec=engage_soft_start_remaining,
        )

    def status_dict(self) -> dict[str, Any]:
        return asdict(self.status_snapshot())

    def _process_control_frame(self, frame, dt_sec: float, receive_time_ns: int):
        if self._engage_state != "IDLE":
            return self._process_relative_engage(frame, dt_sec, receive_time_ns)
        collecting = self.calibrator.state in {CalibrationState.COUNTDOWN, CalibrationState.SAMPLING}
        if not collecting and self.state_machine.state is not TeleopState.RUNNING:
            return None
        try:
            return self.frame_processor(frame, dt_sec, receive_time_ns)
        except PauseControl:
            if collecting and self.calibrator.state is CalibrationState.VALID:
                try:
                    self._activate_calibrated_absolute(
                        frame,
                        auto_start=self._absolute_auto_start_after_calibration,
                    )
                except RuntimeError as exc:
                    self.state_machine.state = TeleopState.PAUSED
                    self._set_error(str(exc))
                finally:
                    self._absolute_auto_start_after_calibration = False
                self._write_calibration_report()
                return None
            elif self.calibrator.state is CalibrationState.INVALID:
                self.state_machine.state = TeleopState.PAUSED
                self._set_error(self.calibrator.failure_reason)
            raise

    @staticmethod
    def _frame_context(frame) -> tuple[str, str, int | None]:
        return (
            frame.xr_session_id,
            frame.session.reference_space,
            frame.session.reference_space_revision,
        )

    def _clear_relative_engage_state(self) -> None:
        self._engage_state = "IDLE"
        self._engage_samples.clear()
        self._engage_started_ns = None
        self._engage_soft_start_started_ns = None
        self._engage_robot_anchors.clear()
        self._engage_context = None
        self._engage_last_sample_key = None

    def _cancel_relative_engage_state(self) -> None:
        if self._engage_state == "IDLE":
            return
        self._clear_relative_engage_state()
        self.mapper.disengage()
        for arm_filter in self.arm_filters.values():
            arm_filter.reset()

    def _abort_relative_engage(self, reason: str) -> None:
        self._clear_relative_engage_state()
        self.mapper.disengage()
        for arm_filter in self.arm_filters.values():
            arm_filter.reset()
        self.state_machine.state = TeleopState.PAUSED
        self.loop.pause(reason)
        self._set_error(reason)
        self._last_command_message = reason

    def _append_relative_engage_sample(self, frame) -> bool:
        if not frame.session.active or not frame.hmd.valid:
            self._engage_samples.clear()
            self._engage_last_sample_key = None
            return False
        if any(not frame.arm_wrists[side].valid for side in self.enabled_sides):
            self._engage_samples.clear()
            self._engage_last_sample_key = None
            return False
        context = self._frame_context(frame)
        if self._engage_context is None:
            self._engage_context = context
        elif context != self._engage_context:
            self._engage_samples.clear()
            self._engage_context = context
        sample_key = (frame.xr_session_id, int(frame.seq))
        if sample_key == self._engage_last_sample_key:
            return len(self._engage_samples) >= self.engage_stable_frames
        self._engage_last_sample_key = sample_key
        self._engage_samples.append(
            {
                "hmd_position": np.asarray(frame.hmd.position, dtype=float),
                "hmd_yaw": yaw_from_quat_y_up(frame.hmd.orientation_xyzw),
                "wrists": {
                    side: self._wrist_pose(frame, side)
                    for side in self.enabled_sides
                },
            }
        )
        return len(self._engage_samples) >= self.engage_stable_frames

    def _relative_engage_window_stable(self) -> bool:
        if len(self._engage_samples) < self.engage_stable_frames:
            return False
        samples = list(self._engage_samples)
        hmd_positions = np.asarray([sample["hmd_position"] for sample in samples], dtype=float)
        hmd_median = np.median(hmd_positions, axis=0)
        if float(np.max(np.linalg.norm(hmd_positions - hmd_median, axis=1))) > self.engage_hmd_position_stability_m:
            return False
        hmd_yaws = np.asarray([sample["hmd_yaw"] for sample in samples], dtype=float)
        mean_yaw = float(np.arctan2(np.mean(np.sin(hmd_yaws)), np.mean(np.cos(hmd_yaws))))
        yaw_errors = np.angle(np.exp(1j * (hmd_yaws - mean_yaw)))
        if float(np.max(np.abs(yaw_errors))) > self.engage_hmd_yaw_stability_rad:
            return False
        for side in self.enabled_sides:
            positions = np.asarray(
                [sample["wrists"][side].position_array() for sample in samples],
                dtype=float,
            )
            median = np.median(positions, axis=0)
            if float(np.max(np.linalg.norm(positions - median, axis=1))) > self.engage_wrist_position_stability_m:
                return False
            mean_orientation = mean_quaternion_xyzw(
                [sample["wrists"][side].orientation_array() for sample in samples]
            )
            if max(
                quat_angle_xyzw(sample["wrists"][side].orientation_array(), mean_orientation)
                for sample in samples
            ) > self.engage_wrist_rotation_stability_rad:
                return False
        return True

    def _finish_relative_engage_stabilization(self, frame, receive_time_ns: int) -> None:
        samples = list(self._engage_samples)
        hmd_positions = np.asarray([sample["hmd_position"] for sample in samples], dtype=float)
        self._engage_operator_position = np.median(hmd_positions, axis=0)
        hmd_yaws = np.asarray([sample["hmd_yaw"] for sample in samples], dtype=float)
        operator_yaw = float(np.arctan2(np.mean(np.sin(hmd_yaws)), np.mean(np.cos(hmd_yaws))))
        wrists = {}
        for side in self.enabled_sides:
            positions = np.asarray(
                [sample["wrists"][side].position_array() for sample in samples],
                dtype=float,
            )
            orientations = [sample["wrists"][side].orientation_array() for sample in samples]
            wrists[side] = ArmTarget(
                np.median(positions, axis=0).tolist(),
                mean_quaternion_xyzw(orientations).tolist(),
            )
        current = self.adapter.get_current_poses(frame="chassis")
        self.mapper.set_relative_operator_yaw(operator_yaw)
        self.frame_processor.reset_tracking_reacquire()
        self.frame_processor.set_orientation_reference_context(frame)
        self._engage_robot_anchors = {
            side: ArmTarget(
                current[side].position_array().tolist(),
                current[side].orientation_array().tolist(),
            )
            for side in self.enabled_sides
        }
        for side in self.enabled_sides:
            self.mapper.recenter(side, wrists[side], current[side])
            self.arm_filters[side].reset(current[side])
        self._engage_state = "SOFT_START"
        self._engage_soft_start_started_ns = int(receive_time_ns)
        self._last_command_message = "engage tracking stabilized; current-pose soft start active"

    def _soft_start_targets(self, frame, dt_sec: float, receive_time_ns: int):
        if self._frame_context(frame) != self._engage_context:
            self._abort_relative_engage("engage aborted: WebXR reference space changed")
            return None
        if not frame.session.active or not frame.hmd.valid:
            self._abort_relative_engage("engage aborted: HMD tracking invalid")
            return None
        invalid = [side for side in self.enabled_sides if not frame.arm_wrists[side].valid]
        if invalid:
            self._abort_relative_engage("engage aborted: arm wrist tracking invalid: " + ", ".join(invalid))
            return None
        started = self._engage_soft_start_started_ns
        if started is None:
            self._abort_relative_engage("engage aborted: soft-start timing is unavailable")
            return None
        elapsed = max(0.0, (int(receive_time_ns) - started) / 1e9)
        if self.engage_hold_duration_sec > 0.0 and elapsed <= self.engage_hold_duration_sec:
            gain = 0.0
        elif self.engage_soft_start_duration_sec <= 0.0:
            gain = 1.0
        else:
            u = float(np.clip(
                (elapsed - self.engage_hold_duration_sec) / self.engage_soft_start_duration_sec,
                0.0,
                1.0,
            ))
            gain = u * u * (3.0 - 2.0 * u)
        targets = {}
        for side in self.enabled_sides:
            mapped = self.mapper.map_hand(side, self._wrist_pose(frame, side))
            anchor = self._engage_robot_anchors[side]
            blended = ArmTarget(
                (
                    anchor.position_array()
                    + gain * (mapped.position_array() - anchor.position_array())
                ).tolist(),
                quat_slerp_xyzw(
                    anchor.orientation_array(),
                    mapped.orientation_array(),
                    gain,
                ).tolist(),
            )
            command = self.arm_filters[side].filter(
                blended,
                valid=True,
                running=True,
                dt_sec=dt_sec,
            )
            if command.enabled:
                targets[side] = command.target
        if gain >= 1.0:
            self._clear_relative_engage_state()
            self.state_machine.state = TeleopState.RUNNING
            self.loop.resume()
            self._last_command_message = "engage complete; teleoperation RUNNING"
            with self._status_lock:
                self._last_error = ""
        return targets or None

    def _process_relative_engage(self, frame, dt_sec: float, receive_time_ns: int):
        if self._engage_state == "STABILIZING" and self._engage_started_ns is not None:
            elapsed = max(0.0, (int(receive_time_ns) - self._engage_started_ns) / 1e9)
            if elapsed > self.engage_timeout_sec:
                self._abort_relative_engage("engage timed out waiting for stable HMD and wrist tracking")
                return None
        if self._engage_state == "STABILIZING":
            if not self._append_relative_engage_sample(frame):
                return None
            if not self._relative_engage_window_stable():
                return None
            self._finish_relative_engage_stabilization(frame, receive_time_ns)
        if self._engage_state == "SOFT_START":
            return self._soft_start_targets(frame, dt_sec, receive_time_ns)
        return None

    def _pump_commands(self, now_ns: int) -> None:
        if self._engage_state != "IDLE" and self.loop.pause_latched:
            self._cancel_relative_engage_state()
        if (
            self.loop.state is LoopState.PAUSED
            and self.loop.pause_latched
            and self.state_machine.state is TeleopState.RUNNING
        ):
            self.state_machine.state = TeleopState.PAUSED
        elif self.loop.state is LoopState.FAULT:
            self.state_machine.state = TeleopState.FAULT
        while True:
            command = self.commands.get_nowait()
            if command is None:
                return
            try:
                accepted, message = self._execute_on_control_thread(command, now_ns)
            except Exception as exc:
                accepted, message = False, str(exc)
                self._set_error(message)
            if command.name != "status":
                self._last_command_message = message
            self.commands.complete(command, accepted, message)

    def _execute_on_control_thread(self, command: ControlCommand, now_ns: int) -> tuple[bool, str]:
        name = command.name
        if name == "status":
            return True, json.dumps(self.status_dict(), ensure_ascii=True, sort_keys=True)
        if name == "calibration-status":
            return True, self._calibration_status_message()
        if name in {"mode", "mapping-mode"}:
            return self._switch_mode(command.argument)
        if name in {"absolute-calibrate"}:
            return self._start_absolute_calibration(now_ns, auto_start=False)
        if name in {"cancel-calibration"}:
            self.frame_processor.cancel_absolute_calibration("operator cancelled calibration")
            self._absolute_auto_start_after_calibration = False
            self.state_machine.state = TeleopState.PAUSED
            self.loop.pause("operator cancelled calibration")
            return True, "absolute calibration cancelled"
        if name in {"invalidate-calibration"}:
            reason = command.argument or "operator invalidated calibration"
            self.frame_processor.invalidate_absolute_calibration(reason)
            if self.mapper.mode is MappingMode.ABSOLUTE:
                self.state_machine.state = TeleopState.PAUSED
                self.loop.pause(reason)
            return True, reason
        if name in {"recenter", "calibrate"}:
            if self.mapper.mode is MappingMode.ABSOLUTE:
                return self._start_absolute_calibration(now_ns, auto_start=False)
            return self._relative_recenter(now_ns)
        if name == "anchor-calibrate":
            return self._fixed_anchor_calibrate(now_ns)
        if name == "anchor-engage":
            return self._fixed_anchor_engage(now_ns)
        if name == "clutch-position":
            return self._fixed_anchor_clutch_position(now_ns)
        if name == "clutch-resume":
            return self._fixed_anchor_clutch_resume(now_ns)
        if name == "recover-init":
            return self._recover_to_init_and_pause()
        if name == "engage":
            return self._engage_for_current_mode(now_ns)
        if name == "start":
            return self._start_running(now_ns)
        if name == "pause":
            self._cancel_relative_engage_state()
            self.state_machine.state = TeleopState.PAUSED
            self.loop.pause("operator pause")
            return True, "teleoperation paused"
        if name == "stop":
            self._cancel_relative_engage_state()
            self.state_machine.state = TeleopState.IDLE
            self.loop.pause("operator stop")
            self.mapper.disengage()
            return True, "teleoperation stopped"
        if name == "estop":
            self._cancel_relative_engage_state()
            self.frame_processor.invalidate_absolute_calibration("software E-stop")
            self.state_machine.estop()
            self.loop.pause("software E-stop")
            return True, "software E-stop active; physical E-stop remains required"
        if name == "reset":
            self._cancel_relative_engage_state()
            if self.loop.state is LoopState.FAULT:
                return False, "control-loop fault requires process restart"
            self.state_machine.reset()
            self.loop.pause("reset to IDLE")
            return True, "state reset to IDLE"
        return False, f"unknown control command: {name}"

    def _relative_recenter(self, now_ns: int) -> tuple[bool, str]:
        if self.fixed_anchor_mode:
            return False, "fixed-anchor mode uses anchor-calibrate or clutch-position"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "recenter requires IDLE, ARMED, or PAUSED"
        frame = self._require_fresh_frame(now_ns)
        self.frame_processor.reset_tracking_reacquire()
        self.frame_processor.set_orientation_reference_context(frame)
        desired = self.adapter.get_desired_poses(frame="chassis")
        for side in self.enabled_sides:
            self.mapper.recenter(side, self._wrist_pose(frame, side), desired[side])
            self.arm_filters[side].reset(desired[side])
        self.state_machine.state = TeleopState.ARMED
        self.loop.pause("relative recenter complete; explicit start required")
        return True, "relative recenter complete; state ARMED, explicit start required"

    def _engage_for_current_mode(self, now_ns: int) -> tuple[bool, str]:
        if self.mapper.mode is MappingMode.ABSOLUTE:
            return self._start_absolute_calibration(now_ns, auto_start=True)
        return self._begin_relative_engage(now_ns)

    def _begin_relative_engage(self, now_ns: int) -> tuple[bool, str]:
        if self.mapper.mode is not MappingMode.RELATIVE:
            return False, "relative engage requires relative mapping mode"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "engage requires IDLE, ARMED, or PAUSED"
        frame = self._require_fresh_frame(now_ns, require_revision=self.fixed_anchor_mode)
        self._require_tracking(frame)
        self._cancel_relative_engage_state()
        self.mapper.disengage()
        for arm_filter in self.arm_filters.values():
            arm_filter.reset()
        self.frame_processor.reset_tracking_reacquire()
        self._engage_state = "STABILIZING"
        self._engage_started_ns = int(now_ns)
        self._engage_context = self._frame_context(frame)
        self.state_machine.state = TeleopState.PAUSED
        self.loop.begin_calibration()
        with self._status_lock:
            self._last_error = ""
        self._append_relative_engage_sample(frame)
        sample_count = len(self._engage_samples)
        if self._relative_engage_window_stable():
            self._finish_relative_engage_stabilization(frame, now_ns)
            if self.engage_hold_duration_sec <= 0.0 and self.engage_soft_start_duration_sec <= 0.0:
                self._soft_start_targets(
                    frame,
                    1.0 / float(self.arm_config.get("control_rate_hz", 100.0)),
                    now_ns,
                )
        return True, (
            f"engage stabilization started: {sample_count}/"
            f"{self.engage_stable_frames} stable HMD/wrist frames; auto-start enabled"
        )

    def _relative_engage(self, now_ns: int) -> tuple[bool, str]:
        if self.mapper.mode is not MappingMode.RELATIVE:
            return False, "engage is available only in relative mode"
        if self.fixed_anchor_mode:
            return False, "fixed-anchor mode uses engage"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "engage requires IDLE, ARMED, or PAUSED"
        frame = self._require_fresh_frame(now_ns)
        self._require_tracking(frame)
        self.frame_processor.reset_tracking_reacquire()
        self.frame_processor.set_orientation_reference_context(frame)
        current = self.adapter.get_current_poses(frame="chassis")
        for side in self.enabled_sides:
            self.mapper.recenter(side, self._wrist_pose(frame, side), current[side])
            self.arm_filters[side].reset(current[side])
        self.state_machine.state = TeleopState.RUNNING
        self.loop.resume()
        return True, "relative current-pose anchor and start completed atomically; teleoperation RUNNING"

    def _fixed_anchor_calibrate(self, now_ns: int) -> tuple[bool, str]:
        if not self.fixed_anchor_mode or self.mapper.mode is not MappingMode.RELATIVE:
            return False, "anchor-calibrate requires fixed-anchor relative mode"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "anchor-calibrate is forbidden while RUNNING"
        frame = self._require_fresh_frame(now_ns, require_revision=True)
        self._require_tracking(frame)
        desired = self.adapter.get_desired_poses(frame="chassis")
        self.frame_processor.reset_tracking_reacquire()
        self.frame_processor.set_orientation_reference_context(frame)
        for side in self.enabled_sides:
            self.mapper.recenter(side, self._wrist_pose(frame, side), desired[side])
            self.arm_filters[side].reset(desired[side])
        self.state_machine.state = TeleopState.ARMED
        self.loop.pause("fixed anchor calibrated; explicit start required")
        return True, "fixed anchor calibrated from current hands and robot desired poses; state ARMED"

    def _fixed_anchor_engage(self, now_ns: int) -> tuple[bool, str]:
        if not self.fixed_anchor_mode or self.mapper.mode is not MappingMode.RELATIVE:
            return False, "anchor-engage requires fixed-anchor relative mode"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "anchor-engage requires IDLE, ARMED, or PAUSED"
        frame = self._require_fresh_frame(now_ns, require_revision=True)
        self._require_tracking(frame)
        current = self.adapter.get_current_poses(frame="chassis")
        self.frame_processor.reset_tracking_reacquire()
        self.frame_processor.set_orientation_reference_context(frame)
        for side in self.enabled_sides:
            self.mapper.recenter(side, self._wrist_pose(frame, side), current[side])
            self.arm_filters[side].reset(current[side])
        self.state_machine.state = TeleopState.RUNNING
        self.loop.resume()
        return True, "fixed anchor calibrated from robot current poses and teleoperation started atomically"

    def _fixed_anchor_clutch_position(self, now_ns: int) -> tuple[bool, str]:
        if not self.fixed_anchor_mode or self.mapper.mode is not MappingMode.RELATIVE:
            return False, "clutch-position requires fixed-anchor relative mode"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "clutch-position is forbidden while RUNNING; pause first"
        if not all(self.mapper.is_calibrated(side) for side in self.enabled_sides):
            return False, "clutch-position requires an existing fixed anchor"
        frame = self._require_fresh_frame(now_ns, require_revision=True)
        self._require_tracking(frame)
        if not self.frame_processor.orientation_reference_matches(frame):
            return False, "position clutch cannot preserve orientation after a reference-space change; engage again"
        desired = self.adapter.get_desired_poses(frame="chassis")
        self.frame_processor.reset_tracking_reacquire()
        for side in self.enabled_sides:
            self.mapper.reanchor_position_only(side, self._wrist_pose(frame, side), desired[side])
            self.arm_filters[side].reset(desired[side])
        self.state_machine.state = TeleopState.ARMED
        self.loop.pause("position clutch complete; fixed orientation anchor preserved")
        return True, "position clutch complete; orientation anchor preserved; state ARMED"

    def _fixed_anchor_clutch_resume(self, now_ns: int) -> tuple[bool, str]:
        if not self.fixed_anchor_mode or self.mapper.mode is not MappingMode.RELATIVE:
            return False, "clutch-resume requires fixed-anchor relative mode"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "clutch-resume is forbidden while RUNNING; pause first"
        if not all(self.mapper.is_calibrated(side) for side in self.enabled_sides):
            return False, "clutch-resume requires an existing fixed anchor"
        frame = self._require_fresh_frame(now_ns, require_revision=True)
        self._require_tracking(frame)
        if not self.frame_processor.orientation_reference_matches(frame):
            return False, "position clutch cannot preserve orientation after a reference-space change; engage again"
        desired = self.adapter.get_desired_poses(frame="chassis")
        for side in self.enabled_sides:
            self.mapper.reanchor_position_only(side, self._wrist_pose(frame, side), desired[side])
        safety = self.arm_config.get("safety", {})
        max_rotation = float(safety.get("mode_switch_max_rotation_jump_rad", 0.35))
        catchup_started = self.frame_processor.prepare_fixed_anchor_clutch_recovery(
            frame,
            desired,
            max_rotation,
        )
        self.state_machine.state = TeleopState.RUNNING
        self.loop.resume()
        if catchup_started:
            return True, "position clutch completed; bounded orientation catch-up started; orientation anchor preserved"
        return True, "position clutch and resume completed atomically; orientation anchor preserved"

    def _recover_to_init_and_pause(self) -> tuple[bool, str]:
        if not self.init_recovery_enabled:
            return False, "recover-init requires --enable-m8-init-recovery"
        if not self.fixed_anchor_mode or self.mapper.mode is not MappingMode.RELATIVE:
            return False, "recover-init requires fixed-anchor relative mode"
        if self.state_machine.state in {TeleopState.ESTOP, TeleopState.FAULT}:
            return False, f"recover-init is unavailable while {self.state_machine.state.value}"
        self._cancel_relative_engage_state()
        arm_targets = self.init_recovery_config.get("arms", {})
        targets = {
            side: list(arm_targets[side])
            for side in self.enabled_sides
            if side in arm_targets
        }
        if set(targets) != set(self.enabled_sides):
            return False, "recover-init configuration is missing an enabled arm target"
        duration = float(self.init_recovery_config.get("duration_sec", 4.0))
        tolerance = float(self.init_recovery_config.get("joint_tolerance_rad", 0.10))
        self.state_machine.state = TeleopState.PAUSED
        self.loop.pause("operator init recovery")
        try:
            self.adapter.move_arms_to_joint_positions(
                targets,
                duration_sec=duration,
                tolerance_rad=tolerance,
            )
        finally:
            self.loop.reset_timing_after_blocking_maintenance()
        self.frame_processor.reset_tracking_reacquire()
        self.mapper.disengage()
        for arm_filter in self.arm_filters.values():
            arm_filter.reset()
        self.state_machine.state = TeleopState.PAUSED
        self.loop.pause("init recovery complete; engage required")
        with self._status_lock:
            self._last_error = ""
        return True, (
            f"arms moved to recorded init joints in {duration:.1f} s; "
            "teleoperation PAUSED; engage required"
        )

    def _start_absolute_calibration(self, now_ns: int, *, auto_start: bool = False) -> tuple[bool, str]:
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "absolute calibration is forbidden while RUNNING"
        frame = self._require_fresh_frame(now_ns, require_revision=True)
        self._require_tracking(frame)
        self.state_machine.state = TeleopState.PAUSED
        self.loop.begin_calibration()
        self._absolute_auto_start_after_calibration = bool(auto_start)
        self.frame_processor.start_absolute_calibration(now_ns)
        if auto_start:
            return True, "absolute calibration started; teleoperation will run automatically after validation"
        return True, "absolute calibration countdown started; no arm commands will be sent"

    def _start_running(self, now_ns: int) -> tuple[bool, str]:
        absolute_resume = (
            self.mapper.mode is MappingMode.ABSOLUTE
            and self.state_machine.state is TeleopState.PAUSED
            and self.calibrator.state is CalibrationState.VALID
        )
        fixed_anchor_resume = (
            self.fixed_anchor_mode
            and self.mapper.mode is MappingMode.RELATIVE
            and self.state_machine.state is TeleopState.PAUSED
            and all(self.mapper.is_calibrated(side) for side in self.enabled_sides)
        )
        if self.state_machine.state is not TeleopState.ARMED and not absolute_resume and not fixed_anchor_resume:
            return False, f"start requires ARMED, got {self.state_machine.state.value}"
        frame = self._require_fresh_frame(now_ns, require_revision=self.mapper.mode is MappingMode.ABSOLUTE)
        self._require_tracking(frame)
        if self.fixed_anchor_mode and not self.frame_processor.orientation_reference_matches(frame):
            return False, "fixed anchor is invalid for the current WebXR reference space; engage again"
        desired = self.adapter.get_desired_poses(frame="chassis")
        candidates = self._candidate_targets(frame)
        safety = self.arm_config.get("safety", {})
        max_position = float(safety.get("mode_switch_max_position_jump_m", 0.05))
        max_rotation = float(safety.get("mode_switch_max_rotation_jump_rad", 0.35))
        for side, candidate in candidates.items():
            self._check_candidate(side, candidate, desired[side], max_position, max_rotation)
            self.arm_filters[side].reset(desired[side])
        self.state_machine.state = TeleopState.RUNNING
        self.loop.resume()
        return True, "teleoperation RUNNING"

    def _activate_calibrated_absolute(self, frame, *, auto_start: bool = False) -> None:
        desired = self.adapter.get_desired_poses(frame="chassis")
        hands = {side: self._wrist_pose(frame, side) for side in self.enabled_sides}
        safety = self.arm_config.get("safety", {})
        max_position = float(safety.get("mode_switch_max_position_jump_m", 0.05))
        max_rotation = float(safety.get("mode_switch_max_rotation_jump_rad", 0.35))
        result = self.frame_processor.switch_mapping_mode(
            MappingMode.ABSOLUTE,
            TeleopState.PAUSED.value,
            hands,
            desired,
            (frame.xr_session_id, frame.session.reference_space, frame.session.reference_space_revision),
            max_position,
            max_rotation,
        )
        if not result.accepted:
            raise RuntimeError(f"absolute calibration valid but activation rejected: {result.reason}")
        for side in self.enabled_sides:
            self.arm_filters[side].reset(desired[side])
        if auto_start:
            self.state_machine.state = TeleopState.RUNNING
            self.loop.resume()
        else:
            self.state_machine.state = TeleopState.ARMED
            self.loop.pause("absolute calibration complete; explicit start required")

    def _switch_mode(self, argument: str | None) -> tuple[bool, str]:
        if argument is None:
            return False, "mode command requires relative or absolute"
        try:
            requested = MappingMode(argument)
        except ValueError:
            return False, "mapping mode must be relative or absolute"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "mapping mode cannot change while RUNNING"
        snapshot = self.tracking_buffer.snapshot()
        frame = None if snapshot is None else snapshot.frame
        hands = None
        desired = None
        context = None
        if requested is MappingMode.ABSOLUTE:
            if frame is None:
                return False, "absolute mode requires a current WebXR frame"
            hands = {side: self._wrist_pose(frame, side) for side in self.enabled_sides}
            desired = self.adapter.get_desired_poses(frame="chassis")
            context = (frame.xr_session_id, frame.session.reference_space, frame.session.reference_space_revision)
        safety = self.arm_config.get("safety", {})
        result = self.frame_processor.switch_mapping_mode(
            requested,
            self.state_machine.state.value,
            hands,
            desired,
            context,
            float(safety.get("mode_switch_max_position_jump_m", 0.05)),
            float(safety.get("mode_switch_max_rotation_jump_rad", 0.35)),
        )
        if not result.accepted:
            return False, result.reason
        self.state_machine.state = TeleopState.PAUSED
        self.loop.pause(f"mapping mode changed to {requested.value}; explicit preparation required")
        return True, f"mapping mode changed to {requested.value}; no command sent"

    def _candidate_targets(self, frame) -> dict[str, ArmTarget]:
        candidates = {}
        for side in self.enabled_sides:
            candidates[side] = self.mapper.map_hand(
                side,
                self._wrist_pose(frame, side),
                xr_session_id=frame.xr_session_id,
                reference_space=frame.session.reference_space,
                reference_space_revision=frame.session.reference_space_revision,
                hmd_valid=frame.hmd.valid,
            )
        return candidates

    def _check_candidate(self, side: str, candidate: ArmTarget, desired: ArmTarget, max_position: float, max_rotation: float) -> None:
        from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_angle_xyzw

        position_jump = float(np.linalg.norm(candidate.position_array() - desired.position_array()))
        rotation_jump = quat_angle_xyzw(candidate.orientation_array(), desired.orientation_array())
        arm_filter = self.arm_filters[side]
        candidate_position = candidate.position_array()
        violations = []
        for index, axis in enumerate(("x", "y", "z")):
            if candidate_position[index] < arm_filter.xyz_min[index]:
                violations.append(
                    f"{axis}={candidate_position[index]:.4f} below {arm_filter.xyz_min[index]:.4f}"
                )
            elif candidate_position[index] > arm_filter.xyz_max[index]:
                violations.append(
                    f"{axis}={candidate_position[index]:.4f} above {arm_filter.xyz_max[index]:.4f}"
                )
        if violations:
            raise RuntimeError(
                f"{side} start candidate is outside configured workspace: "
                + "; ".join(violations)
            )
        if position_jump > max_position or rotation_jump > max_rotation:
            raise RuntimeError(
                f"{side} start jump ({position_jump:.4f} m, {rotation_jump:.4f} rad) exceeds limits"
            )

    def _require_fresh_frame(self, now_ns: int, require_revision: bool = False):
        snapshot = self.tracking_buffer.snapshot()
        if snapshot is None:
            raise RuntimeError("no WebXR tracking frame received")
        age_ns = max(0, int(now_ns) - snapshot.receive_time_ns)
        if age_ns > self.loop.fresh_timeout_ns:
            raise RuntimeError(f"WebXR frame is stale ({age_ns / 1e6:.1f} ms)")
        frame = snapshot.frame
        if not frame.session.active:
            raise RuntimeError("WebXR session is inactive")
        if require_revision and frame.session.reference_space_revision is None:
            raise RuntimeError("reference_space_revision is required")
        return frame

    def _require_tracking(self, frame) -> None:
        if not frame.hmd.valid:
            raise RuntimeError("HMD tracking is invalid")
        invalid = [side for side in self.enabled_sides if not frame.arm_wrists[side].valid]
        if invalid:
            raise RuntimeError(f"arm wrist tracking invalid: {', '.join(invalid)}")

    @staticmethod
    def _wrist_pose(frame, side: str) -> ArmTarget:
        wrist = frame.arm_wrists[side]
        if not wrist.valid:
            raise RuntimeError(f"{side} arm wrist tracking invalid")
        return ArmTarget(wrist.position, wrist.orientation_xyzw)

    def _seed_dryrun_robot_poses(self) -> None:
        arm_entries = self.arm_config.get("arms", {})
        poses = {}
        for side in ("left", "right"):
            minimum = np.asarray(arm_entries.get(side, {}).get("workspace_xyz_min", [0, 0, 0]), dtype=float)
            maximum = np.asarray(arm_entries.get(side, {}).get("workspace_xyz_max", [0, 0, 0]), dtype=float)
            poses[side] = ArmTarget(((minimum + maximum) * 0.5).tolist(), [0, 0, 0, 1])
        self.adapter.set_dryrun_poses(desired=poses, current=poses)

    def _calibration_status_message(self) -> str:
        progress = self.calibrator.progress(monotonic_ns())
        reason = self.calibrator.failure_reason
        return f"{progress['state']} samples={progress['sample_count']}/{progress['minimum_valid_samples']}" + (f" reason={reason}" if reason else "")

    def _write_calibration_report(self) -> None:
        if self._calibration_report is None or self.calibrator.result is None:
            return
        result = self.calibrator.result
        report = {
            "valid_for_runtime_only": True,
            "restorable": False,
            "xr_session_id": result.xr_session_id,
            "reference_space": result.reference_space,
            "reference_space_revision": result.reference_space_revision,
            "start_time_ns": result.start_time_ns,
            "end_time_ns": result.end_time_ns,
            "enabled_sides": list(result.enabled_sides),
            "quality": dict(result.quality),
        }
        self._calibration_report.parent.mkdir(parents=True, exist_ok=True)
        self._calibration_report.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")

    def _set_error(self, message: str) -> None:
        with self._status_lock:
            self._last_error = str(message)


class DryRunSupervisor:
    def __init__(self):
        self.state_machine = TeleopStateMachine()
        self.converter = WebXRToMP21Converter()
        self.hand_retarget = {side: RetargetPipeline(dry_run=True) for side in ("left", "right")}
        self.hand_filters = {side: HandSafetyFilter(lower=[0.0] * 20, upper=[1.0] * 20) for side in ("left", "right")}
        self.arm_mapper = ArmMapper(position_scale=1.0)
        self.arm_filters = {side: ArmSafetyFilter(max_position_delta=0.25) for side in ("left", "right")}

    def handle_command(self, command: str) -> None:
        if command.strip().lower() == "calibrate":
            self.state_machine.arm()
        else:
            self.state_machine.handle_command(command)

    def process_payload(self, payload: dict[str, Any]) -> SupervisorOutput:
        frame = validate_tracking_frame(payload)
        if not frame.session.active:
            self.arm_mapper.invalidate_absolute("WebXR session ended")
            self.state_machine.pause()
        self._ensure_calibrated(frame)
        running = self.state_machine.state is TeleopState.RUNNING
        hands: dict[str, HandCommand] = {}
        arms: dict[str, ArmCommand] = {}
        for side in ("left", "right"):
            hand = frame.hands[side]
            mp21 = self.converter.convert(hand)
            qpos = self.hand_retarget[side].retarget(mp21)
            hands[side] = self.hand_filters[side].filter(
                qpos,
                valid=hand.valid and frame.hmd.valid and frame.session.active,
                now_sec=frame.client_time_sec,
                running=running,
            )
            webxr_target = self._hand_arm_target(hand)
            mapped = self.arm_mapper.map_hand(
                side,
                webxr_target,
                xr_session_id=frame.xr_session_id,
                reference_space=frame.session.reference_space,
                reference_space_revision=frame.session.reference_space_revision,
                hmd_valid=frame.hmd.valid,
            )
            arms[side] = self.arm_filters[side].filter(
                mapped,
                valid=hand.valid and frame.hmd.valid and frame.session.active,
                running=running,
            )
        return SupervisorOutput(state=self.state_machine.state.value, hands=hands, arms=arms, seq=frame.seq)

    def _ensure_calibrated(self, frame) -> None:
        for side in ("left", "right"):
            if not self.arm_mapper.is_calibrated(side):
                self.arm_mapper.calibrate(
                    side,
                    ArmTarget([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                    self._hand_arm_target(frame.hands[side]),
                )

    def _hand_arm_target(self, hand) -> ArmTarget:
        if not hand.valid or not hand.positions:
            return ArmTarget([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        wrist_idx = hand.joint_names.index("wrist") if "wrist" in hand.joint_names else 0
        return ArmTarget(list(hand.positions[wrist_idx]), list(hand.orientations_xyzw[wrist_idx]))

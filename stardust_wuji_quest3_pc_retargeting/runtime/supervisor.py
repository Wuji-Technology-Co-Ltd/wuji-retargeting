from __future__ import annotations

import json
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
    position_scale_xyz: tuple[float, float, float]
    max_linear_speed_mps: float
    max_input_position_jump_m: float
    start_max_position_jump_m: float
    position_alpha: float
    enable_orientation: bool
    rotation_scale: float
    max_angular_speed_rad_s: float
    hand_reacquire_timeout_sec: float
    hand_reacquire_state: str
    hand_reacquire_sides: tuple[str, ...]
    hand_reacquire_remaining_sec: float
    absolute_orientation_reacquire: bool
    orientation_reacquire_speed_rad_s: float
    reacquire_position_errors_m: dict[str, float]
    reacquire_orientation_errors_rad: dict[str, float]


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
    ) -> None:
        if arm not in {"left", "right", "both"}:
            raise ValueError("arm must be left, right, or both")
        self.arm_config = arm_config
        self.enabled_sides = ("left", "right") if arm == "both" else (arm,)
        mapping = arm_config.get("mapping", {})
        selected_mode = MappingMode(mapping_mode or mapping.get("mode", "relative"))
        if enable_real_arm and (arm not in {"left", "both"} or selected_mode is not MappingMode.RELATIVE):
            raise RuntimeError("M8 real arm is limited to left or both arms in relative mode")
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
        )
        self.tracking_buffer = LatestTrackingBuffer()
        self.commands = ControlCommandQueue()
        self.state_machine = TeleopStateMachine()
        self._status_lock = Lock()
        self._latest_frame = None
        self._last_error = ""
        self._last_command_message = ""
        self._activate_absolute_after_calibration = False
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
            position_scale_xyz=tuple(float(value) for value in self.mapper.position_scale_xyz),
            max_linear_speed_mps=float(next(iter(self.arm_filters.values())).max_linear_speed_mps),
            max_input_position_jump_m=float(next(iter(self.arm_filters.values())).max_input_position_jump_m),
            start_max_position_jump_m=float(self.arm_config.get("safety", {}).get("mode_switch_max_position_jump_m", 0.05)),
            position_alpha=float(self.arm_config.get("filter", {}).get("position_alpha", 1.0)),
            enable_orientation=bool(self.mapper.enable_orientation),
            rotation_scale=float(self.mapper.rotation_scale),
            max_angular_speed_rad_s=float(next(iter(self.arm_filters.values())).max_angular_speed_rad_s),
            hand_reacquire_timeout_sec=float(self.frame_processor.hand_reacquire_timeout_sec),
            hand_reacquire_state=str(hand_reacquire["state"]),
            hand_reacquire_sides=tuple(hand_reacquire["sides"]),
            hand_reacquire_remaining_sec=float(hand_reacquire["remaining_sec"]),
            absolute_orientation_reacquire=bool(self.frame_processor.absolute_orientation_reacquire),
            orientation_reacquire_speed_rad_s=float(
                self.frame_processor.orientation_reacquire_speed_rad_s
            ),
            reacquire_position_errors_m=dict(hand_reacquire["position_errors_m"]),
            reacquire_orientation_errors_rad=dict(hand_reacquire["orientation_errors_rad"]),
        )

    def status_dict(self) -> dict[str, Any]:
        return asdict(self.status_snapshot())

    def _process_control_frame(self, frame, dt_sec: float, receive_time_ns: int):
        collecting = self.calibrator.state in {CalibrationState.COUNTDOWN, CalibrationState.SAMPLING}
        if not collecting and self.state_machine.state is not TeleopState.RUNNING:
            return None
        try:
            return self.frame_processor(frame, dt_sec, receive_time_ns)
        except PauseControl:
            if self.calibrator.state is CalibrationState.VALID:
                if self._activate_absolute_after_calibration:
                    try:
                        self._activate_calibrated_absolute(frame)
                    except RuntimeError as exc:
                        self.state_machine.state = TeleopState.PAUSED
                        self._set_error(str(exc))
                    finally:
                        self._activate_absolute_after_calibration = False
                else:
                    self.state_machine.state = TeleopState.ARMED
                self._write_calibration_report()
            elif self.calibrator.state is CalibrationState.INVALID:
                self.state_machine.state = TeleopState.PAUSED
                self._set_error(self.calibrator.failure_reason)
            raise

    def _pump_commands(self, now_ns: int) -> None:
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
            return self._start_absolute_calibration(now_ns)
        if name in {"cancel-calibration"}:
            self.frame_processor.cancel_absolute_calibration("operator cancelled calibration")
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
                return self._start_absolute_calibration(now_ns)
            return self._relative_recenter(now_ns)
        if name == "engage":
            return self._relative_engage(now_ns)
        if name == "start":
            return self._start_running(now_ns)
        if name == "pause":
            self.state_machine.state = TeleopState.PAUSED
            self.loop.pause("operator pause")
            return True, "teleoperation paused"
        if name == "stop":
            self.state_machine.state = TeleopState.IDLE
            self.loop.pause("operator stop")
            self.mapper.disengage()
            return True, "teleoperation stopped"
        if name == "estop":
            self.frame_processor.invalidate_absolute_calibration("software E-stop")
            self.state_machine.estop()
            self.loop.pause("software E-stop")
            return True, "software E-stop active; physical E-stop remains required"
        if name == "reset":
            if self.loop.state is LoopState.FAULT:
                return False, "control-loop fault requires process restart"
            self.state_machine.reset()
            self.loop.pause("reset to IDLE")
            return True, "state reset to IDLE"
        return False, f"unknown control command: {name}"

    def _relative_recenter(self, now_ns: int) -> tuple[bool, str]:
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

    def _relative_engage(self, now_ns: int) -> tuple[bool, str]:
        if self.mapper.mode is not MappingMode.RELATIVE:
            return False, "engage is available only in relative mode"
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "engage requires IDLE, ARMED, or PAUSED"
        frame = self._require_fresh_frame(now_ns)
        self._require_tracking(frame)
        self.frame_processor.reset_tracking_reacquire()
        self.frame_processor.set_orientation_reference_context(frame)
        desired = self.adapter.get_desired_poses(frame="chassis")
        for side in self.enabled_sides:
            self.mapper.recenter(side, self._wrist_pose(frame, side), desired[side])
            self.arm_filters[side].reset(desired[side])
        self.state_machine.state = TeleopState.RUNNING
        self.loop.resume()
        return True, "relative recenter and start completed atomically; teleoperation RUNNING"

    def _start_absolute_calibration(self, now_ns: int) -> tuple[bool, str]:
        if self.state_machine.state is TeleopState.RUNNING:
            return False, "absolute calibration is forbidden while RUNNING"
        frame = self._require_fresh_frame(now_ns, require_revision=True)
        self._require_tracking(frame)
        self.state_machine.state = TeleopState.PAUSED
        self.loop.begin_calibration()
        self._activate_absolute_after_calibration = True
        self.frame_processor.start_absolute_calibration(now_ns)
        return True, "absolute calibration countdown started; no arm commands will be sent"

    def _start_running(self, now_ns: int) -> tuple[bool, str]:
        absolute_resume = (
            self.mapper.mode is MappingMode.ABSOLUTE
            and self.state_machine.state is TeleopState.PAUSED
            and self.calibrator.state is CalibrationState.VALID
        )
        if self.state_machine.state is not TeleopState.ARMED and not absolute_resume:
            return False, f"start requires ARMED, got {self.state_machine.state.value}"
        frame = self._require_fresh_frame(now_ns, require_revision=self.mapper.mode is MappingMode.ABSOLUTE)
        self._require_tracking(frame)
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

    def _activate_calibrated_absolute(self, frame) -> None:
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
        if np.any(candidate.position_array() < arm_filter.xyz_min) or np.any(candidate.position_array() > arm_filter.xyz_max):
            raise RuntimeError(f"{side} start candidate is outside configured workspace")
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
        invalid = [side for side in self.enabled_sides if not frame.hands[side].valid or "wrist" not in frame.hands[side].joint_names]
        if invalid:
            raise RuntimeError(f"hand tracking invalid: {', '.join(invalid)}")

    @staticmethod
    def _wrist_pose(frame, side: str) -> ArmTarget:
        hand = frame.hands[side]
        if not hand.valid or "wrist" not in hand.joint_names:
            raise RuntimeError(f"{side} wrist tracking invalid")
        index = hand.joint_names.index("wrist")
        return ArmTarget(hand.positions[index], hand.orientations_xyzw[index])

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

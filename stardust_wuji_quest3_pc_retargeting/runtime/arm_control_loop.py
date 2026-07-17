from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from threading import Event, Thread, get_ident
from time import monotonic_ns, sleep
from typing import Callable, Mapping, Protocol

import numpy as np

from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmMapper, ArmTarget, MappingMode, ModeSwitchResult
from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import (
    AbsoluteCalibrationSample,
    AbsoluteSessionCalibrator,
    CalibrationError,
    CalibrationState,
    PoseSample,
)
from stardust_wuji_quest3_pc_retargeting.protocol.messages import TrackingFrame
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_angle_xyzw, quat_slerp_xyzw
from stardust_wuji_quest3_pc_retargeting.runtime.latest_tracking import LatestTrackingBuffer, TrackingSnapshot
from stardust_wuji_quest3_pc_retargeting.safety.arm_safety_filter import ArmSafetyFilter


class ArmAdapter(Protocol):
    def initialize(self) -> None: ...

    def send_targets(self, targets: Mapping[str, ArmTarget]) -> None: ...

    def get_desired_poses(self, frame: str = "chassis") -> Mapping[str, ArmTarget]: ...

    def get_current_poses(self, frame: str = "chassis") -> Mapping[str, ArmTarget]: ...

    def close(self) -> None: ...


class LoopState(str, Enum):
    STOPPED = "STOPPED"
    ACTIVE = "ACTIVE"
    HOLD = "HOLD"
    PAUSED = "PAUSED"
    FAULT = "FAULT"


class PauseControl(RuntimeError):
    def __init__(self, reason: str, latch: bool = True):
        super().__init__(reason)
        self.latch = bool(latch)


@dataclass
class ArmLoopStats:
    cycles: int = 0
    sent_cycles: int = 0
    consumed_frames: int = 0
    held_cycles: int = 0
    stale_pauses: int = 0
    missed_deadlines: int = 0
    loop_period_ns: list[int] = field(default_factory=list)
    mapper_time_ns: list[int] = field(default_factory=list)
    sdk_call_time_ns: list[int] = field(default_factory=list)
    sdk_call_interval_ns: list[int] = field(default_factory=list)
    frame_age_ns: list[int] = field(default_factory=list)
    target_linear_speeds_mps: list[float] = field(default_factory=list)
    target_angular_speeds_rad_s: list[float] = field(default_factory=list)
    paused_hold_cycles: int = 0

    def record(self, series: list[int], value: int, maximum_samples: int = 10000) -> None:
        series.append(int(value))
        if len(series) > maximum_samples:
            del series[: len(series) - maximum_samples]


class ArmControlLoop:
    def __init__(
        self,
        adapter: ArmAdapter,
        tracking_buffer: LatestTrackingBuffer,
        frame_processor: Callable[[object, float], Mapping[str, ArmTarget] | None],
        control_rate_hz: float = 100.0,
        fresh_timeout_sec: float = 0.05,
        disable_timeout_sec: float = 0.10,
        sdk_block_fault_sec: float = 0.020,
        consecutive_deadline_fault_count: int = 3,
        clock_ns: Callable[[], int] = monotonic_ns,
        sleeper: Callable[[float], None] = sleep,
        command_pump: Callable[[int], None] | None = None,
    ) -> None:
        self.adapter = adapter
        self.tracking_buffer = tracking_buffer
        self.frame_processor = frame_processor
        self.control_rate_hz = float(control_rate_hz)
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        self.period_ns = int(round(1e9 / self.control_rate_hz))
        self.fresh_timeout_ns = int(float(fresh_timeout_sec) * 1e9)
        self.disable_timeout_ns = int(float(disable_timeout_sec) * 1e9)
        if not (0 < self.fresh_timeout_ns < self.disable_timeout_ns):
            raise ValueError("fresh timeout must be positive and below disable timeout")
        self.sdk_block_fault_ns = int(float(sdk_block_fault_sec) * 1e9)
        self.consecutive_deadline_fault_count = int(consecutive_deadline_fault_count)
        self.clock_ns = clock_ns
        self.sleeper = sleeper
        self.command_pump = command_pump
        self.stats = ArmLoopStats()
        self.state = LoopState.STOPPED
        self.fault_reason = ""
        self._last_generation = 0
        self._last_frame_key: tuple[object, object] | None = None
        self._last_targets: dict[str, ArmTarget] = {}
        self._last_sent_targets: dict[str, tuple[int, ArmTarget]] = {}
        self._last_tick_ns: int | None = None
        self._consecutive_deadlines = 0
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._owner_thread_id: int | None = None
        self._pause_latched = False
        self._paused_hold_targets: dict[str, ArmTarget] = {}
        self._last_sdk_call_ns: int | None = None
        self._deadline_reset_requested = False

    @property
    def pause_latched(self) -> bool:
        return self._pause_latched

    @property
    def paused_hold_active(self) -> bool:
        return bool(self._paused_hold_targets)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = Thread(target=self.run, name="ArmControlLoop", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._cancel_calibration("control loop stopped")
        self._stop_event.set()
        if self._thread is not None and self._thread.ident != get_ident():
            self._thread.join(timeout=timeout)

    def resume(self) -> None:
        if self.state is LoopState.FAULT:
            raise RuntimeError("faulted control loop must be reset before resume")
        self._pause_latched = False
        self._paused_hold_targets.clear()
        self.state = LoopState.HOLD

    def begin_calibration(self, *, keep_paused_hold: bool = False) -> None:
        if self.state is LoopState.FAULT:
            raise RuntimeError("faulted control loop cannot calibrate")
        self._pause_latched = False
        self._last_targets.clear()
        if not keep_paused_hold:
            self._paused_hold_targets.clear()
        self.state = LoopState.PAUSED

    def pause(self, reason: str = "operator pause") -> None:
        self._cancel_calibration(reason)
        self._pause_latched = True
        self._last_targets.clear()
        self._last_sent_targets.clear()
        self._paused_hold_targets.clear()
        self.state = LoopState.PAUSED
        self.fault_reason = reason

    def pause_with_hold(
        self,
        reason: str = "operator pause hold",
        targets: Mapping[str, ArmTarget] | None = None,
    ) -> None:
        self._assert_owner()
        self._cancel_calibration(reason)
        source = self._last_targets if targets is None else targets
        if source:
            self._paused_hold_targets = {
                side: self._copy_target(target)
                for side, target in source.items()
            }
        self._pause_latched = True
        self._last_targets.clear()
        self.state = LoopState.PAUSED
        self.fault_reason = reason

    def set_paused_hold_targets(self, targets: Mapping[str, ArmTarget]) -> None:
        self._assert_owner()
        self._paused_hold_targets = {
            side: self._copy_target(target)
            for side, target in targets.items()
        }

    def establish_paused_hold(
        self,
        targets: Mapping[str, ArmTarget],
        duration_sec: float,
    ) -> int:
        self._assert_owner()
        duration = float(duration_sec)
        if not 0.10 <= duration <= 3.0:
            raise ValueError("Cartesian handoff hold duration must be in [0.10, 3.0] seconds")
        copied = {
            side: self._copy_target(target)
            for side, target in targets.items()
        }
        if not copied:
            raise ValueError("Cartesian handoff requires at least one arm target")
        self._paused_hold_targets = copied
        self._pause_latched = True
        self._last_targets.clear()
        self.state = LoopState.PAUSED
        cycles = max(1, int(np.ceil(duration * self.control_rate_hz)))
        for index in range(cycles):
            self._send(copied, self.clock_ns())
            if self.state is LoopState.FAULT:
                raise RuntimeError(self.fault_reason or "Cartesian handoff failed")
            self.stats.paused_hold_cycles += 1
            if index + 1 < cycles:
                self.sleeper(1.0 / self.control_rate_hz)
        return cycles

    def reset_timing_after_blocking_maintenance(self) -> None:
        self._assert_owner()
        self._last_tick_ns = None
        self._consecutive_deadlines = 0
        self._deadline_reset_requested = True

    def fail_closed(self, reason: str) -> None:
        self._assert_owner()
        self._fault(str(reason))

    def run(self, max_cycles: int | None = None) -> None:
        if self._owner_thread_id is not None:
            raise RuntimeError("arm control loop is already running")
        self._owner_thread_id = get_ident()
        try:
            self.adapter.initialize()
            self.state = LoopState.HOLD
            next_deadline = self.clock_ns()
            while not self._stop_event.is_set() and (max_cycles is None or self.stats.cycles < max_cycles):
                now = self.clock_ns()
                if now < next_deadline:
                    self.sleeper((next_deadline - now) / 1e9)
                    now = self.clock_ns()
                elif now - next_deadline >= self.period_ns:
                    self.stats.missed_deadlines += max(1, (now - next_deadline) // self.period_ns)
                self.tick(now)
                if self._deadline_reset_requested:
                    self._deadline_reset_requested = False
                    next_deadline = self.clock_ns() + self.period_ns
                    continue
                next_deadline += self.period_ns
                if now >= next_deadline:
                    next_deadline = now + self.period_ns
        except Exception as exc:
            self._fault(f"control loop exception: {exc}")
        finally:
            self.adapter.close()
            if self.state is not LoopState.FAULT:
                self.state = LoopState.STOPPED
            self._owner_thread_id = None

    def tick(self, now_ns: int | None = None) -> LoopState:
        self._assert_owner()
        now = self.clock_ns() if now_ns is None else int(now_ns)
        dt_sec = self.period_ns / 1e9 if self._last_tick_ns is None else max(0, now - self._last_tick_ns) / 1e9
        if self._last_tick_ns is not None:
            self.stats.record(self.stats.loop_period_ns, now - self._last_tick_ns)
        self._last_tick_ns = now
        self.stats.cycles += 1
        if self.command_pump is not None:
            try:
                self.command_pump(now)
            except Exception as exc:
                self._fault(f"command pump failed: {exc}")
                return self.state
        if self.state is LoopState.FAULT:
            return self.state
        if self._pause_latched:
            self.state = LoopState.PAUSED
            if self._paused_hold_targets:
                self._send(self._paused_hold_targets, now)
                self.stats.paused_hold_cycles += 1
            self.stats.held_cycles += 1
            return self.state
        snapshot = self.tracking_buffer.snapshot()
        if snapshot is None:
            self.state = LoopState.PAUSED if self._paused_hold_targets else LoopState.HOLD
            if self._paused_hold_targets:
                self._send(self._paused_hold_targets, now)
                self.stats.paused_hold_cycles += 1
            self.stats.held_cycles += 1
            return self.state
        age_ns = max(0, now - snapshot.receive_time_ns)
        self.stats.record(self.stats.frame_age_ns, age_ns)
        if age_ns > self.disable_timeout_ns:
            if self.state is not LoopState.PAUSED:
                self.stats.stale_pauses += 1
            self.pause("tracking frame timeout")
            self.stats.held_cycles += 1
            return self.state
        frame_key = self._frame_key(snapshot.frame)
        has_new_frame = (
            snapshot.generation != self._last_generation
            and frame_key != self._last_frame_key
            and age_ns <= self.fresh_timeout_ns
        )
        active_candidate = False
        if has_new_frame:
            consumed = self._consume(snapshot, dt_sec)
            if consumed is None:
                return self.state
            active_candidate = consumed
        else:
            self.stats.held_cycles += 1
            self.state = LoopState.PAUSED if self._paused_hold_targets else LoopState.HOLD
        if self._last_targets:
            self._send(self._last_targets, now)
            if self.state not in {LoopState.FAULT, LoopState.PAUSED}:
                self.state = LoopState.ACTIVE if active_candidate else LoopState.HOLD
        elif self._paused_hold_targets:
            self._send(self._paused_hold_targets, now)
            self.stats.paused_hold_cycles += 1
            self.state = LoopState.PAUSED
        return self.state

    def _consume(self, snapshot: TrackingSnapshot, dt_sec: float) -> bool | None:
        started = self.clock_ns()
        try:
            targets = self.frame_processor(snapshot.frame, dt_sec, snapshot.receive_time_ns)
        except PauseControl as exc:
            self._last_generation = snapshot.generation
            self.stats.consumed_frames += 1
            self._last_targets.clear()
            if exc.latch:
                self.pause(str(exc))
            else:
                self.state = LoopState.PAUSED
                self.fault_reason = str(exc)
            return False
        except Exception as exc:
            self._fault(f"frame processor failed: {exc}")
            return None
        self.stats.record(self.stats.mapper_time_ns, max(0, self.clock_ns() - started))
        self._last_generation = snapshot.generation
        self._last_frame_key = self._frame_key(snapshot.frame)
        self.stats.consumed_frames += 1
        if targets:
            self._last_targets.update({side: self._copy_target(target) for side, target in targets.items()})
        return bool(targets)

    def _send(self, targets: Mapping[str, ArmTarget], command_time_ns: int) -> None:
        self._record_target_speeds(targets, command_time_ns)
        if self._last_sdk_call_ns is not None:
            self.stats.record(
                self.stats.sdk_call_interval_ns,
                max(0, int(command_time_ns) - self._last_sdk_call_ns),
            )
        self._last_sdk_call_ns = int(command_time_ns)
        started = self.clock_ns()
        try:
            self.adapter.send_targets(targets)
        except Exception as exc:
            self._fault(f"Astribot SDK call failed: {exc}")
            return
        elapsed = max(0, self.clock_ns() - started)
        self.stats.record(self.stats.sdk_call_time_ns, elapsed)
        self.stats.sent_cycles += 1
        if elapsed > self.period_ns:
            self.stats.missed_deadlines += 1
        if elapsed > self.sdk_block_fault_ns:
            self._consecutive_deadlines += 1
            if self._consecutive_deadlines >= self.consecutive_deadline_fault_count:
                self._fault(f"Astribot SDK blocked for {elapsed / 1e6:.3f} ms")
        else:
            self._consecutive_deadlines = 0

    def _record_target_speeds(self, targets: Mapping[str, ArmTarget], command_time_ns: int) -> None:
        from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_angle_xyzw

        for side, target in targets.items():
            previous = self._last_sent_targets.get(side)
            if previous is not None:
                previous_time_ns, previous_target = previous
                dt = max(1e-9, (int(command_time_ns) - previous_time_ns) / 1e9)
                linear_speed = float(np.linalg.norm(target.position_array() - previous_target.position_array()) / dt)
                angular_speed = quat_angle_xyzw(target.orientation_array(), previous_target.orientation_array()) / dt
                self.stats.target_linear_speeds_mps.append(linear_speed)
                self.stats.target_angular_speeds_rad_s.append(angular_speed)
            self._last_sent_targets[side] = (int(command_time_ns), self._copy_target(target))

    def _assert_owner(self) -> None:
        thread_id = get_ident()
        if self._owner_thread_id is None:
            self._owner_thread_id = thread_id
        elif self._owner_thread_id != thread_id:
            raise RuntimeError("all ArmControlLoop operations must run on its owner thread")

    def _fault(self, reason: str) -> None:
        self._invalidate_calibration(reason)
        self.fault_reason = reason
        self.state = LoopState.FAULT
        self._stop_event.set()
        call_times = self.stats.sdk_call_time_ns
        max_call_ms = 0.0 if not call_times else max(call_times) / 1e6
        p50_call_ms = 0.0 if not call_times else float(np.percentile(call_times, 50)) / 1e6
        p95_call_ms = 0.0 if not call_times else float(np.percentile(call_times, 95)) / 1e6
        print(
            f"ARM CONTROL LOOP FAULT: {reason}; sent_cycles={self.stats.sent_cycles}; "
            f"missed_deadlines={self.stats.missed_deadlines}; sdk_call_p50_ms={p50_call_ms:.3f}; "
            f"sdk_call_p95_ms={p95_call_ms:.3f}; max_sdk_call_ms={max_call_ms:.3f}",
            flush=True,
        )

    def _cancel_calibration(self, reason: str) -> None:
        cancel = getattr(self.frame_processor, "cancel_absolute_calibration", None)
        if callable(cancel):
            cancel(reason)

    def _invalidate_calibration(self, reason: str) -> None:
        invalidate = getattr(self.frame_processor, "invalidate_absolute_calibration", None)
        if callable(invalidate):
            invalidate(reason)
        else:
            self._cancel_calibration(reason)

    @staticmethod
    def _copy_target(target: ArmTarget) -> ArmTarget:
        return ArmTarget(target.position_array().tolist(), target.orientation_array().tolist())

    @staticmethod
    def _frame_key(frame: object) -> tuple[object, object]:
        if isinstance(frame, dict):
            return frame.get("xr_session_id"), frame.get("seq")
        return getattr(frame, "xr_session_id", None), getattr(frame, "seq", None)


class ArmFrameProcessor:
    def __init__(
        self,
        mapper: ArmMapper,
        safety_filters: Mapping[str, ArmSafetyFilter],
        enabled_sides=("left", "right"),
        adapter: ArmAdapter | None = None,
        calibrator: AbsoluteSessionCalibrator | None = None,
        hand_reacquire_timeout_sec: float = 0.0,
        hand_reacquire_stable_frames: int = 3,
        hand_reacquire_invalid_grace_frames: int = 2,
        absolute_orientation_reacquire: bool = False,
        orientation_reacquire_speed_rad_s: float = 0.5,
        orientation_reacquire_direct_error_rad: float = 0.15,
        orientation_reacquire_complete_error_rad: float = 0.087,
        orientation_reacquire_max_error_rad: float = 1.57,
        orientation_reacquire_complete_frames: int = 5,
        absolute_pose_reacquire: bool = False,
        absolute_reacquire_linear_speed_mps: float = 0.10,
        absolute_reacquire_direct_position_error_m: float = 0.02,
        absolute_reacquire_complete_position_error_m: float = 0.01,
        absolute_reacquire_max_position_error_m: float = 0.20,
        fixed_anchor_pose_reacquire: bool = False,
        pose_reacquire_linear_accel_mps2: float = 0.30,
        pose_reacquire_angular_accel_rad_s2: float = 1.50,
    ) -> None:
        self.mapper = mapper
        self.safety_filters = dict(safety_filters)
        self.enabled_sides = tuple(enabled_sides)
        self.adapter = adapter
        self.calibrator = calibrator
        self.hand_reacquire_timeout_sec = float(hand_reacquire_timeout_sec)
        self.hand_reacquire_stable_frames = int(hand_reacquire_stable_frames)
        self.hand_reacquire_invalid_grace_frames = int(hand_reacquire_invalid_grace_frames)
        self.absolute_orientation_reacquire = bool(absolute_orientation_reacquire)
        self.orientation_reacquire_speed_rad_s = float(orientation_reacquire_speed_rad_s)
        self.orientation_reacquire_direct_error_rad = float(orientation_reacquire_direct_error_rad)
        self.orientation_reacquire_complete_error_rad = float(orientation_reacquire_complete_error_rad)
        self.orientation_reacquire_max_error_rad = float(orientation_reacquire_max_error_rad)
        self.orientation_reacquire_complete_frames = int(orientation_reacquire_complete_frames)
        self.absolute_pose_reacquire = bool(absolute_pose_reacquire)
        self.absolute_reacquire_linear_speed_mps = float(absolute_reacquire_linear_speed_mps)
        self.absolute_reacquire_direct_position_error_m = float(absolute_reacquire_direct_position_error_m)
        self.absolute_reacquire_complete_position_error_m = float(absolute_reacquire_complete_position_error_m)
        self.absolute_reacquire_max_position_error_m = float(absolute_reacquire_max_position_error_m)
        self.fixed_anchor_pose_reacquire = bool(fixed_anchor_pose_reacquire)
        self.pose_reacquire_linear_accel_mps2 = float(pose_reacquire_linear_accel_mps2)
        self.pose_reacquire_angular_accel_rad_s2 = float(pose_reacquire_angular_accel_rad_s2)
        if not np.isfinite(self.hand_reacquire_timeout_sec) or self.hand_reacquire_timeout_sec < 0.0:
            raise ValueError("hand reacquire timeout must be finite and non-negative")
        if self.hand_reacquire_stable_frames < 1:
            raise ValueError("hand reacquire stable frames must be positive")
        if self.hand_reacquire_invalid_grace_frames < 0:
            raise ValueError("hand reacquire invalid grace frames must be non-negative")
        if not np.isfinite(self.orientation_reacquire_speed_rad_s) or self.orientation_reacquire_speed_rad_s <= 0.0:
            raise ValueError("orientation reacquire speed must be finite and positive")
        if not (
            0.0 < self.orientation_reacquire_complete_error_rad
            <= self.orientation_reacquire_direct_error_rad
            <= self.orientation_reacquire_max_error_rad
            <= np.pi
        ):
            raise ValueError("orientation reacquire error thresholds must be positive and ordered through pi")
        if self.orientation_reacquire_complete_frames < 1:
            raise ValueError("orientation reacquire complete frames must be positive")
        if not np.isfinite(self.absolute_reacquire_linear_speed_mps) or self.absolute_reacquire_linear_speed_mps <= 0.0:
            raise ValueError("absolute reacquire linear speed must be finite and positive")
        if not (
            0.0 < self.absolute_reacquire_complete_position_error_m
            <= self.absolute_reacquire_direct_position_error_m
            <= self.absolute_reacquire_max_position_error_m
        ):
            raise ValueError("absolute reacquire position thresholds must be positive and ordered")
        if not np.isfinite(self.pose_reacquire_linear_accel_mps2) or self.pose_reacquire_linear_accel_mps2 <= 0.0:
            raise ValueError("pose reacquire linear acceleration must be finite and positive")
        if not np.isfinite(self.pose_reacquire_angular_accel_rad_s2) or self.pose_reacquire_angular_accel_rad_s2 <= 0.0:
            raise ValueError("pose reacquire angular acceleration must be finite and positive")
        if self.hand_reacquire_timeout_sec > 0.0 and self.adapter is None:
            raise ValueError("automatic hand reacquire requires an arm adapter")
        self._tracking_loss_started_ns: int | None = None
        self._tracking_loss_sides: tuple[str, ...] = ()
        self._tracking_reacquire_valid_frames = 0
        self._tracking_invalid_frames = 0
        self._tracking_reacquire_timed_out = False
        self._tracking_alignment_required = False
        self._orientation_catchup_active = False
        self._orientation_catchup_targets: dict[str, ArmTarget] = {}
        self._orientation_catchup_complete_frames = 0
        self._reacquire_position_errors_m: dict[str, float] = {}
        self._reacquire_orientation_errors_rad: dict[str, float] = {}
        self._reacquire_candidate_positions_m: dict[str, list[float]] = {}
        self._reacquire_workspace_violations: dict[str, list[str]] = {}
        self._last_filter_rejections: dict[str, str] = {}
        self._recovery_trigger_reason = ""
        self._tracking_loss_events = 0
        self._tracking_catchup_interruptions = 0
        self._tracking_recovery_completions = 0
        self._orientation_catchup_linear_velocities: dict[str, np.ndarray] = {}
        self._orientation_catchup_angular_speeds: dict[str, float] = {}
        self._orientation_reference_context: tuple[str, str, int | None] | None = None
        if any(side not in {"left", "right"} or side not in self.safety_filters for side in self.enabled_sides):
            raise ValueError("enabled sides require matching left/right safety filters")

    def reset_tracking_reacquire(self) -> None:
        self._tracking_loss_started_ns = None
        self._tracking_loss_sides = ()
        self._tracking_reacquire_valid_frames = 0
        self._tracking_invalid_frames = 0
        self._tracking_reacquire_timed_out = False
        self._tracking_alignment_required = False
        self._orientation_catchup_active = False
        self._orientation_catchup_targets.clear()
        self._orientation_catchup_complete_frames = 0
        self._orientation_catchup_linear_velocities.clear()
        self._orientation_catchup_angular_speeds.clear()
        self._reacquire_position_errors_m.clear()
        self._reacquire_orientation_errors_rad.clear()
        self._reacquire_candidate_positions_m.clear()
        self._reacquire_workspace_violations.clear()
        self._last_filter_rejections.clear()
        self._recovery_trigger_reason = ""

    def set_orientation_reference_context(self, frame: TrackingFrame) -> None:
        self._orientation_reference_context = (
            frame.xr_session_id,
            frame.session.reference_space,
            frame.session.reference_space_revision,
        )

    def orientation_reference_matches(self, frame: TrackingFrame) -> bool:
        return self._orientation_reference_context == (
            frame.xr_session_id,
            frame.session.reference_space,
            frame.session.reference_space_revision,
        )

    def tracking_reacquire_status(self, now_ns: int) -> dict[str, object]:
        common = {
            "position_errors_m": dict(self._reacquire_position_errors_m),
            "orientation_errors_rad": dict(self._reacquire_orientation_errors_rad),
            "candidate_positions_m": dict(self._reacquire_candidate_positions_m),
            "workspace_violations": dict(self._reacquire_workspace_violations),
            "filter_rejections": dict(self._last_filter_rejections),
            "trigger_reason": self._recovery_trigger_reason,
            "loss_events": self._tracking_loss_events,
            "catchup_interruptions": self._tracking_catchup_interruptions,
            "recovery_completions": self._tracking_recovery_completions,
        }
        if self._orientation_catchup_active:
            return {
                "state": (
                    "ABSOLUTE_POSE_CATCHUP"
                    if self.mapper.mode is MappingMode.ABSOLUTE
                    else (
                        "FIXED_ANCHOR_CATCHUP"
                        if self.fixed_anchor_pose_reacquire
                        else "ORIENTATION_CATCHUP"
                    )
                ),
                "sides": self.enabled_sides,
                "remaining_sec": 0.0,
                **common,
            }
        if self._tracking_alignment_required:
            elapsed_sec = 0.0 if self._tracking_loss_started_ns is None else max(
                0.0, (int(now_ns) - self._tracking_loss_started_ns) / 1e9
            )
            return {
                "state": "ALIGNMENT_REQUIRED",
                "sides": self._tracking_loss_sides,
                "remaining_sec": max(0.0, self.hand_reacquire_timeout_sec - elapsed_sec),
                **common,
            }
        if self._tracking_reacquire_timed_out:
            return {"state": "TIMED_OUT", "sides": self._tracking_loss_sides, "remaining_sec": 0.0, **common}
        if self._tracking_loss_started_ns is None:
            return {"state": "IDLE", "sides": (), "remaining_sec": 0.0, **common}
        elapsed_sec = max(0.0, (int(now_ns) - self._tracking_loss_started_ns) / 1e9)
        remaining_sec = max(0.0, self.hand_reacquire_timeout_sec - elapsed_sec)
        state = "STABILIZING" if self._tracking_reacquire_valid_frames else "WAITING"
        return {"state": state, "sides": self._tracking_loss_sides, "remaining_sec": remaining_sec, **common}

    def __call__(self, frame: TrackingFrame, dt_sec: float, receive_time_ns: int) -> dict[str, ArmTarget] | None:
        if not frame.session.active:
            self.invalidate_absolute_calibration("WebXR session ended")
            raise PauseControl("WebXR session inactive")
        if not frame.hmd.valid:
            self.invalidate_absolute_calibration("HMD tracking invalid")
            raise PauseControl("HMD tracking invalid")
        if (
            self.absolute_orientation_reacquire or self.fixed_anchor_pose_reacquire
        ) and self._orientation_reference_context is not None:
            current_context = (
                frame.xr_session_id,
                frame.session.reference_space,
                frame.session.reference_space_revision,
            )
            if current_context != self._orientation_reference_context:
                self.reset_tracking_reacquire()
                raise PauseControl("absolute orientation reference space changed; engage again")
        if self.calibrator is not None and self.calibrator.state in {CalibrationState.COUNTDOWN, CalibrationState.SAMPLING}:
            self._collect_calibration_sample(frame, receive_time_ns)
            raise PauseControl("absolute calibration in progress", latch=False)
        invalid_sides = [side for side in self.enabled_sides if not frame.arm_wrists[side].valid]
        if invalid_sides:
            self._handle_invalid_hand_tracking(invalid_sides, receive_time_ns)
        self._tracking_invalid_frames = 0
        if self._orientation_catchup_active:
            return self._process_orientation_catchup(frame, dt_sec, receive_time_ns)
        elif self._tracking_loss_started_ns is not None:
            self._handle_reacquired_hand_tracking(frame, receive_time_ns)
            if self._orientation_catchup_active:
                return self._process_orientation_catchup(frame, dt_sec, receive_time_ns)
        targets: dict[str, ArmTarget] = {}
        for side in self.enabled_sides:
            wrist = frame.arm_wrists[side]
            if not wrist.valid:
                self.safety_filters[side].filter(
                    ArmTarget([0, 0, 0], [0, 0, 0, 1]), valid=False, running=True, dt_sec=dt_sec
                )
                continue
            webxr_pose = ArmTarget(wrist.position, wrist.orientation_xyzw)
            mapped = self.mapper.map_hand(
                side,
                webxr_pose,
                xr_session_id=frame.xr_session_id,
                reference_space=frame.session.reference_space,
                reference_space_revision=frame.session.reference_space_revision,
                hmd_valid=frame.hmd.valid,
            )
            command = self.safety_filters[side].filter(mapped, valid=True, running=True, dt_sec=dt_sec)
            if command.enabled:
                targets[side] = command.target
            elif self.fixed_anchor_pose_reacquire and command.reason.startswith("input "):
                self._begin_discontinuity_recovery(side, command.reason, receive_time_ns)
                raise PauseControl(
                    f"{side} tracking discontinuity detected; starting fixed-anchor recovery: "
                    f"{command.reason}",
                    latch=False,
                )
            elif self.mapper.mode is MappingMode.ABSOLUTE and command.reason.startswith("input "):
                self.mapper.invalidate_absolute(f"{side} tracking origin discontinuity: {command.reason}")
                raise PauseControl(f"{side} absolute calibration invalidated: {command.reason}")
        return targets or None

    def _begin_discontinuity_recovery(
        self,
        side: str,
        reason: str,
        receive_time_ns: int,
    ) -> None:
        self._orientation_catchup_active = False
        self._orientation_catchup_targets.clear()
        self._orientation_catchup_complete_frames = 0
        self._orientation_catchup_linear_velocities.clear()
        self._orientation_catchup_angular_speeds.clear()
        self._tracking_loss_started_ns = int(receive_time_ns)
        self._tracking_loss_sides = (side,)
        self._tracking_reacquire_valid_frames = 0
        self._tracking_invalid_frames = 0
        self._tracking_reacquire_timed_out = False
        self._tracking_alignment_required = False
        self._reacquire_position_errors_m.clear()
        self._reacquire_orientation_errors_rad.clear()
        self._reacquire_candidate_positions_m.clear()
        self._reacquire_workspace_violations.clear()
        self._last_filter_rejections = {side: str(reason)}
        self._recovery_trigger_reason = f"{side}: {reason}"
        self._tracking_loss_events += 1

    def _handle_invalid_hand_tracking(self, invalid_sides: list[str], receive_time_ns: int) -> None:
        if self.hand_reacquire_timeout_sec <= 0.0:
            if len(self.enabled_sides) > 1:
                raise PauseControl("dual-arm tracking invalid: " + ", ".join(invalid_sides))
            return
        if self.mapper.mode is MappingMode.ABSOLUTE and not self.absolute_pose_reacquire:
            raise PauseControl("automatic absolute-pose reacquire is disabled")
        self._tracking_invalid_frames += 1
        if (
            self._orientation_catchup_active
            and self._tracking_invalid_frames <= self.hand_reacquire_invalid_grace_frames
        ):
            raise PauseControl(
                "brief arm-wrist dropout during catch-up: " + ", ".join(invalid_sides),
                latch=False,
            )
        if self._orientation_catchup_active:
            self._tracking_catchup_interruptions += 1
        self._orientation_catchup_active = False
        self._orientation_catchup_targets.clear()
        self._orientation_catchup_complete_frames = 0
        self._orientation_catchup_linear_velocities.clear()
        self._orientation_catchup_angular_speeds.clear()
        self._tracking_alignment_required = False
        now_ns = int(receive_time_ns)
        if self._tracking_loss_started_ns is None:
            self._tracking_loss_started_ns = now_ns
            self._tracking_loss_events += 1
        self._tracking_loss_sides = tuple(invalid_sides)
        self._tracking_reacquire_valid_frames = 0
        elapsed_sec = max(0.0, (now_ns - self._tracking_loss_started_ns) / 1e9)
        if elapsed_sec > self.hand_reacquire_timeout_sec:
            self._tracking_reacquire_timed_out = True
            raise PauseControl(
                f"hand tracking reacquire timed out after {self.hand_reacquire_timeout_sec:.3f} s"
            )
        raise PauseControl(
            "hand tracking reacquire hold: " + ", ".join(invalid_sides),
            latch=False,
        )

    def _handle_reacquired_hand_tracking(self, frame: TrackingFrame, receive_time_ns: int) -> None:
        assert self._tracking_loss_started_ns is not None
        elapsed_sec = max(0.0, (int(receive_time_ns) - self._tracking_loss_started_ns) / 1e9)
        if elapsed_sec > self.hand_reacquire_timeout_sec:
            self._tracking_reacquire_timed_out = True
            raise PauseControl(
                f"hand tracking reacquire timed out after {self.hand_reacquire_timeout_sec:.3f} s"
            )
        self._tracking_reacquire_valid_frames += 1
        if self._tracking_reacquire_valid_frames < self.hand_reacquire_stable_frames:
            raise PauseControl(
                f"hand tracking reacquire stabilizing "
                f"{self._tracking_reacquire_valid_frames}/{self.hand_reacquire_stable_frames}",
                latch=False,
            )
        if self.adapter is None:
            raise PauseControl("automatic hand reacquire requires an arm adapter")
        desired = self.adapter.get_desired_poses(frame="chassis")
        wrists: dict[str, ArmTarget] = {}
        candidates: dict[str, ArmTarget] = {}
        for side in self.enabled_sides:
            arm_wrist = frame.arm_wrists[side]
            wrist = ArmTarget(arm_wrist.position, arm_wrist.orientation_xyzw)
            wrists[side] = wrist
            candidates[side] = self._map_reacquire_candidate(frame, side, wrist)
            self._reacquire_candidate_positions_m[side] = candidates[side].position_array().tolist()
            self._reacquire_position_errors_m[side] = float(
                np.linalg.norm(candidates[side].position_array() - desired[side].position_array())
            )
            self._reacquire_orientation_errors_rad[side] = quat_angle_xyzw(
                candidates[side].orientation_array(), desired[side].orientation_array()
            )
            if self.mapper.mode is MappingMode.RELATIVE and self.fixed_anchor_pose_reacquire:
                pass
            elif self.mapper.mode is MappingMode.RELATIVE and self.absolute_orientation_reacquire:
                self.mapper.reanchor_position_only(side, wrist, desired[side])
            elif self.mapper.mode is MappingMode.RELATIVE:
                self.mapper.recenter(side, wrist, desired[side])
            self.safety_filters[side].reset(desired[side])
        recovery_enabled = (
            self.mapper.mode is MappingMode.ABSOLUTE and self.absolute_pose_reacquire
        ) or (
            self.mapper.mode is MappingMode.RELATIVE and self.absolute_orientation_reacquire
        ) or (
            self.mapper.mode is MappingMode.RELATIVE and self.fixed_anchor_pose_reacquire
        )
        if not recovery_enabled:
            self.reset_tracking_reacquire()
            return
        maximum_position_error = max(self._reacquire_position_errors_m.values(), default=0.0)
        maximum_error = max(self._reacquire_orientation_errors_rad.values(), default=0.0)
        position_direct = (
            (
                self.mapper.mode is MappingMode.RELATIVE
                and not self.fixed_anchor_pose_reacquire
            )
            or maximum_position_error <= self.absolute_reacquire_direct_position_error_m
        )
        if position_direct and maximum_error <= self.orientation_reacquire_direct_error_rad:
            self.reset_tracking_reacquire()
            return
        self._reacquire_workspace_violations = self._workspace_violations(candidates)
        candidate_outside_workspace = bool(self._reacquire_workspace_violations)
        if (
            maximum_error > self.orientation_reacquire_max_error_rad
            or (
                (
                    self.mapper.mode is MappingMode.ABSOLUTE
                    or self.fixed_anchor_pose_reacquire
                )
                and maximum_position_error > self.absolute_reacquire_max_position_error_m
            )
            or candidate_outside_workspace
        ):
            self._tracking_alignment_required = True
            raise PauseControl(
                "absolute pose recovery requires alignment; "
                f"max position error {maximum_position_error:.3f} m, "
                f"max orientation error {maximum_error:.3f} rad",
                latch=False,
            )
        self._tracking_alignment_required = False
        self._orientation_catchup_active = True
        self._orientation_catchup_targets = {
            side: ArmTarget(desired[side].position_array().tolist(), desired[side].orientation_array().tolist())
            for side in self.enabled_sides
        }
        self._orientation_catchup_linear_velocities = {
            side: np.zeros(3, dtype=float) for side in self.enabled_sides
        }
        self._orientation_catchup_angular_speeds = {
            side: 0.0 for side in self.enabled_sides
        }
        self._orientation_catchup_complete_frames = 0
        self._tracking_loss_started_ns = None
        self._tracking_reacquire_valid_frames = 0
        self._tracking_invalid_frames = 0

    def _process_orientation_catchup(
        self,
        frame: TrackingFrame,
        dt_sec: float,
        receive_time_ns: int,
    ) -> dict[str, ArmTarget] | None:
        candidates: dict[str, ArmTarget] = {}
        wrists: dict[str, ArmTarget] = {}
        for side in self.enabled_sides:
            arm_wrist = frame.arm_wrists[side]
            wrist = ArmTarget(arm_wrist.position, arm_wrist.orientation_xyzw)
            wrists[side] = wrist
            candidates[side] = self._map_reacquire_candidate(frame, side, wrist)
            self._reacquire_candidate_positions_m[side] = candidates[side].position_array().tolist()
        current_position_errors = {
            side: float(
                np.linalg.norm(
                    self._orientation_catchup_targets[side].position_array()
                    - candidates[side].position_array()
                )
            )
            for side in self.enabled_sides
        }
        current_errors = {
            side: quat_angle_xyzw(
                self._orientation_catchup_targets[side].orientation_array(),
                candidates[side].orientation_array(),
            )
            for side in self.enabled_sides
        }
        self._reacquire_workspace_violations = self._workspace_violations(candidates)
        candidate_outside_workspace = bool(self._reacquire_workspace_violations)
        if (
            max(current_errors.values(), default=0.0) > self.orientation_reacquire_max_error_rad
            or (
                (
                    self.mapper.mode is MappingMode.ABSOLUTE
                    or self.fixed_anchor_pose_reacquire
                )
                and max(current_position_errors.values(), default=0.0)
                > self.absolute_reacquire_max_position_error_m
            )
            or candidate_outside_workspace
        ):
            self._orientation_catchup_active = False
            self._tracking_alignment_required = True
            self._tracking_loss_started_ns = int(receive_time_ns)
            self._tracking_loss_sides = self.enabled_sides
            self._reacquire_position_errors_m = current_position_errors
            self._reacquire_orientation_errors_rad = current_errors
            raise PauseControl("absolute pose recovery exceeded the automatic catch-up limits", latch=False)
        targets: dict[str, ArmTarget] = {}
        updated_errors: dict[str, float] = {}
        dt = float(dt_sec)
        for side in self.enabled_sides:
            current = self._orientation_catchup_targets[side]
            candidate = candidates[side]
            position = current.position_array()
            if self.mapper.mode is MappingMode.ABSOLUTE or self.fixed_anchor_pose_reacquire:
                position_delta = candidate.position_array() - position
                position_distance = float(np.linalg.norm(position_delta))
                desired_velocity = np.zeros(3, dtype=float)
                if position_distance > 0.0:
                    desired_speed = min(
                        self.absolute_reacquire_linear_speed_mps,
                        position_distance / max(dt, 1e-9),
                    )
                    desired_velocity = position_delta * (desired_speed / position_distance)
                velocity = desired_velocity
                if self.fixed_anchor_pose_reacquire:
                    previous_velocity = self._orientation_catchup_linear_velocities[side]
                    velocity_delta = desired_velocity - previous_velocity
                    max_velocity_delta = self.pose_reacquire_linear_accel_mps2 * dt
                    velocity_delta_norm = float(np.linalg.norm(velocity_delta))
                    if velocity_delta_norm > max_velocity_delta and velocity_delta_norm > 0.0:
                        velocity_delta *= max_velocity_delta / velocity_delta_norm
                    velocity = previous_velocity + velocity_delta
                position_step = velocity * dt
                if float(np.linalg.norm(position_step)) > position_distance and position_distance > 0.0:
                    position_step = position_delta
                    velocity = np.zeros(3, dtype=float)
                position = position + position_step
                self._orientation_catchup_linear_velocities[side] = velocity
            error = current_errors[side]
            desired_angular_speed = min(
                self.orientation_reacquire_speed_rad_s,
                error / max(dt, 1e-9),
            )
            angular_speed = desired_angular_speed
            if self.fixed_anchor_pose_reacquire:
                previous_angular_speed = self._orientation_catchup_angular_speeds[side]
                angular_speed_delta = self.pose_reacquire_angular_accel_rad_s2 * dt
                angular_speed = min(
                    desired_angular_speed,
                    previous_angular_speed + angular_speed_delta,
                )
                if desired_angular_speed < previous_angular_speed:
                    angular_speed = max(
                        desired_angular_speed,
                        previous_angular_speed - angular_speed_delta,
                    )
            self._orientation_catchup_angular_speeds[side] = angular_speed
            max_angle_step = angular_speed * dt
            fraction = 1.0 if error <= max_angle_step or error <= 0.0 else max_angle_step / error
            orientation = quat_slerp_xyzw(
                current.orientation_array(), candidate.orientation_array(), fraction
            )
            step_target = ArmTarget(position.tolist(), orientation.tolist())
            command = self.safety_filters[side].filter(
                step_target, valid=True, running=True, dt_sec=dt_sec
            )
            if not command.enabled:
                raise PauseControl(f"{side} orientation catch-up rejected: {command.reason}")
            targets[side] = command.target
            self._orientation_catchup_targets[side] = command.target
            updated_errors[side] = quat_angle_xyzw(
                command.target.orientation_array(), candidate.orientation_array()
            )
            current_position_errors[side] = float(
                np.linalg.norm(command.target.position_array() - candidate.position_array())
            )
        self._reacquire_position_errors_m = current_position_errors
        self._reacquire_orientation_errors_rad = updated_errors
        orientation_complete = all(
            error <= self.orientation_reacquire_complete_error_rad
            for error in updated_errors.values()
        )
        position_complete = (
            (
                self.mapper.mode is MappingMode.RELATIVE
                and not self.fixed_anchor_pose_reacquire
            )
            or all(
                error <= self.absolute_reacquire_complete_position_error_m
                for error in current_position_errors.values()
            )
        )
        if orientation_complete and position_complete:
            self._orientation_catchup_complete_frames += 1
        else:
            self._orientation_catchup_complete_frames = 0
        if self._orientation_catchup_complete_frames >= self.orientation_reacquire_complete_frames:
            for side in self.enabled_sides:
                if self.mapper.mode is MappingMode.RELATIVE and not self.fixed_anchor_pose_reacquire:
                    self.mapper.reanchor_position_only(side, wrists[side], targets[side])
                self.safety_filters[side].reset(targets[side])
            self._tracking_recovery_completions += 1
            self.reset_tracking_reacquire()
        return targets or None

    def _workspace_violations(self, candidates: Mapping[str, ArmTarget]) -> dict[str, list[str]]:
        axis_names = ("x", "y", "z")
        violations: dict[str, list[str]] = {}
        for side, candidate in candidates.items():
            position = candidate.position_array()
            safety_filter = self.safety_filters[side]
            side_violations = []
            for index, axis in enumerate(axis_names):
                if position[index] < safety_filter.xyz_min[index]:
                    side_violations.append(
                        f"{axis} below min ({position[index]:.4f} < {safety_filter.xyz_min[index]:.4f})"
                    )
                elif position[index] > safety_filter.xyz_max[index]:
                    side_violations.append(
                        f"{axis} above max ({position[index]:.4f} > {safety_filter.xyz_max[index]:.4f})"
                    )
            if side_violations:
                violations[side] = side_violations
        return violations

    def _map_reacquire_candidate(
        self,
        frame: TrackingFrame,
        side: str,
        wrist: ArmTarget,
    ) -> ArmTarget:
        if self.mapper.mode is MappingMode.ABSOLUTE:
            return self.mapper.map_hand(
                side,
                wrist,
                xr_session_id=frame.xr_session_id,
                reference_space=frame.session.reference_space,
                reference_space_revision=frame.session.reference_space_revision,
                hmd_valid=frame.hmd.valid,
            )
        return self.mapper.map_hand(side, wrist)

    def prepare_fixed_anchor_clutch_recovery(
        self,
        frame: TrackingFrame,
        desired: Mapping[str, ArmTarget],
        normal_rotation_limit_rad: float,
    ) -> bool:
        if not self.fixed_anchor_pose_reacquire or self.mapper.mode is not MappingMode.RELATIVE:
            raise RuntimeError("fixed-anchor clutch recovery requires fixed-anchor relative mode")
        self.reset_tracking_reacquire()
        candidates: dict[str, ArmTarget] = {}
        for side in self.enabled_sides:
            wrist = frame.arm_wrists[side]
            if not wrist.valid:
                raise RuntimeError(f"{side} arm wrist tracking invalid")
            candidates[side] = self.mapper.map_hand(
                side,
                ArmTarget(wrist.position, wrist.orientation_xyzw),
            )
            self._reacquire_candidate_positions_m[side] = candidates[side].position_array().tolist()
            self._reacquire_position_errors_m[side] = float(
                np.linalg.norm(candidates[side].position_array() - desired[side].position_array())
            )
            self._reacquire_orientation_errors_rad[side] = quat_angle_xyzw(
                candidates[side].orientation_array(), desired[side].orientation_array()
            )
            self.safety_filters[side].reset(desired[side])

        self._reacquire_workspace_violations = self._workspace_violations(candidates)
        maximum_position_error = max(self._reacquire_position_errors_m.values(), default=0.0)
        maximum_orientation_error = max(self._reacquire_orientation_errors_rad.values(), default=0.0)
        if self._reacquire_workspace_violations:
            self._tracking_alignment_required = True
            self._tracking_loss_sides = tuple(self._reacquire_workspace_violations)
            raise RuntimeError(
                "clutch-resume candidate is outside configured workspace: "
                + str(self._reacquire_workspace_violations)
            )
        if maximum_position_error > self.absolute_reacquire_max_position_error_m:
            self._tracking_alignment_required = True
            self._tracking_loss_sides = self.enabled_sides
            raise RuntimeError(
                f"clutch-resume position error {maximum_position_error:.4f} m exceeds "
                f"recovery limit {self.absolute_reacquire_max_position_error_m:.4f} m"
            )
        if maximum_orientation_error > self.orientation_reacquire_max_error_rad:
            self._tracking_alignment_required = True
            self._tracking_loss_sides = self.enabled_sides
            raise RuntimeError(
                f"clutch-resume orientation error {maximum_orientation_error:.4f} rad exceeds "
                f"recovery limit {self.orientation_reacquire_max_error_rad:.4f} rad; use engage"
            )
        if maximum_orientation_error <= float(normal_rotation_limit_rad):
            self.reset_tracking_reacquire()
            for side in self.enabled_sides:
                self.safety_filters[side].reset(desired[side])
            return False

        self._orientation_catchup_active = True
        self._orientation_catchup_targets = {
            side: ArmTarget(
                desired[side].position_array().tolist(),
                desired[side].orientation_array().tolist(),
            )
            for side in self.enabled_sides
        }
        self._orientation_catchup_linear_velocities = {
            side: np.zeros(3, dtype=float) for side in self.enabled_sides
        }
        self._orientation_catchup_angular_speeds = {
            side: 0.0 for side in self.enabled_sides
        }
        self._orientation_catchup_complete_frames = 0
        self._tracking_loss_sides = tuple(
            side
            for side, error in self._reacquire_orientation_errors_rad.items()
            if error > float(normal_rotation_limit_rad)
        )
        self._recovery_trigger_reason = (
            f"clutch-resume orientation catch-up from {maximum_orientation_error:.4f} rad"
        )
        return True

    def start_absolute_calibration(self, now_ns: int) -> None:
        if self.calibrator is None or self.adapter is None:
            raise RuntimeError("absolute calibration requires a calibrator and arm adapter")
        self.calibrator.start(now_ns, self.enabled_sides, self.mapper.robot_from_vr_axes)

    def cancel_absolute_calibration(self, reason: str = "calibration cancelled") -> None:
        if self.calibrator is not None and self.calibrator.state in {CalibrationState.COUNTDOWN, CalibrationState.SAMPLING}:
            self.calibrator.cancel(reason)

    def invalidate_absolute_calibration(self, reason: str) -> None:
        self.mapper.invalidate_absolute(reason)
        if self.calibrator is not None and self.calibrator.state is not CalibrationState.UNCALIBRATED:
            self.calibrator.invalidate(reason)

    def _collect_calibration_sample(self, frame: TrackingFrame, receive_time_ns: int) -> None:
        if self.calibrator is None or self.adapter is None:
            raise RuntimeError("absolute calibration collector is not configured")
        if frame.session.reference_space_revision is None:
            self.calibrator.invalidate("reference_space_revision is required")
            raise PauseControl("absolute calibration requires reference_space_revision")
        invalid_sides = [side for side in self.enabled_sides if not frame.arm_wrists[side].valid]
        if invalid_sides:
            reason = f"tracking invalid during calibration: {', '.join(invalid_sides)}"
            self.calibrator.invalidate(reason)
            raise PauseControl(reason)
        desired = self.adapter.get_desired_poses(frame="chassis")
        current = self.adapter.get_current_poses(frame="chassis")
        hands = {}
        for side in self.enabled_sides:
            wrist = frame.arm_wrists[side]
            hands[side] = PoseSample(
                tuple(wrist.position),
                tuple(wrist.orientation_xyzw),
            )
        sample = AbsoluteCalibrationSample(
            receive_time_ns=int(receive_time_ns),
            xr_session_id=frame.xr_session_id,
            reference_space=frame.session.reference_space,
            reference_space_revision=frame.session.reference_space_revision,
            hmd=PoseSample(tuple(frame.hmd.position), tuple(frame.hmd.orientation_xyzw)),
            hands=hands,
            robot_desired={side: PoseSample.from_pose(desired[side]) for side in self.enabled_sides},
            robot_current={side: PoseSample.from_pose(current[side]) for side in self.enabled_sides},
        )
        try:
            state = self.calibrator.add_sample(sample)
        except CalibrationError as exc:
            raise PauseControl(str(exc)) from exc
        if state is CalibrationState.VALID and self.calibrator.result is not None:
            self.mapper.set_absolute_calibration(self.calibrator.result)
            raise PauseControl("absolute calibration complete; explicit start required")

    def switch_mapping_mode(
        self,
        new_mode: MappingMode | str,
        teleop_state: str,
        current_hands: Mapping[str, ArmTarget] | None = None,
        robot_desired: Mapping[str, ArmTarget] | None = None,
        session_context: tuple[str, str, int | None] | None = None,
        max_position_jump_m: float = 0.05,
        max_rotation_jump_rad: float = 0.35,
    ) -> ModeSwitchResult:
        def validate_workspace(side: str, candidate: ArmTarget) -> str | None:
            safety_filter = self.safety_filters[side]
            position = candidate.position_array()
            if not all(position >= safety_filter.xyz_min) or not all(position <= safety_filter.xyz_max):
                return f"{side} candidate is outside configured workspace"
            return None

        result = self.mapper.switch_mode(
            new_mode,
            teleop_state,
            current_hands,
            robot_desired,
            session_context,
            max_position_jump_m,
            max_rotation_jump_rad,
            validate_workspace,
        )
        if result.accepted:
            for safety_filter in self.safety_filters.values():
                safety_filter.reset()
        return result

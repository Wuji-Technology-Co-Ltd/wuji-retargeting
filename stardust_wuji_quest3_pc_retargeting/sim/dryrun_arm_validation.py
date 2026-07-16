from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from threading import Lock
from typing import Sequence

import numpy as np
import yaml

from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmTarget
from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import (
    AbsoluteCalibrationConfig,
    AbsoluteCalibrationSample,
    CalibrationState,
    PoseSample,
    build_absolute_calibration,
)
from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmMapper
from stardust_wuji_quest3_pc_retargeting.arm_control.astribot_adapter import AstribotAdapter
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_angle_xyzw
from stardust_wuji_quest3_pc_retargeting.runtime.config import load_yaml_config
from stardust_wuji_quest3_pc_retargeting.runtime.supervisor import ControlPCSupervisor
from stardust_wuji_quest3_pc_retargeting.sim.mock_webxr_sender import build_mock_frame


@dataclass
class TelemetryRow:
    elapsed_sec: float
    seq: int
    scenario: str
    teleop_state: str
    loop_state: str
    frame_age_ms: float
    sent_cycles: int
    missed_deadlines: int
    left_x: float
    left_y: float
    left_z: float
    left_speed_mps: float
    left_angular_speed_rad_s: float
    right_x: float
    right_y: float
    right_z: float
    right_speed_mps: float
    right_angular_speed_rad_s: float


class DelayInjectingDryRunAdapter(AstribotAdapter):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._delay_sec = 0.0
        self._delay_lock = Lock()
        self.delay_call_counts: dict[int, int] = {}
        self.delay_observed_ms: dict[int, list[float]] = {}
        self.command_linear_speeds: list[float] = []
        self.command_angular_speeds: list[float] = []
        self._previous_commands: dict[str, tuple[int, ArmTarget]] = {}

    def reset_velocity_history(self) -> None:
        with self._delay_lock:
            self._previous_commands.clear()

    def set_delay(self, delay_sec: float) -> None:
        with self._delay_lock:
            self._delay_sec = max(0.0, float(delay_sec))

    @property
    def delay_sec(self) -> float:
        with self._delay_lock:
            return self._delay_sec

    def send_targets(self, targets) -> None:
        delay = self.delay_sec
        started = time.monotonic()
        if delay:
            time.sleep(delay)
        super().send_targets(targets)
        sent_ns = time.monotonic_ns()
        with self._delay_lock:
            for side, target in targets.items():
                previous = self._previous_commands.get(side)
                if previous is not None:
                    previous_ns, previous_target = previous
                    dt = max(1e-9, (sent_ns - previous_ns) / 1e9)
                    self.command_linear_speeds.append(
                        float(np.linalg.norm(target.position_array() - previous_target.position_array()) / dt)
                    )
                    self.command_angular_speeds.append(
                        quat_angle_xyzw(target.orientation_array(), previous_target.orientation_array()) / dt
                    )
                self._previous_commands[side] = (sent_ns, target)
        delay_ms = int(round(delay * 1000.0))
        if delay_ms:
            elapsed_ms = (time.monotonic() - started) * 1000.0
            with self._delay_lock:
                self.delay_call_counts[delay_ms] = self.delay_call_counts.get(delay_ms, 0) + 1
                self.delay_observed_ms.setdefault(delay_ms, []).append(elapsed_ms)


def percentile_ms(values: list[int], percentile: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=float), percentile) / 1e6)


def build_validation_frame(seq: int, elapsed: float, revision: int = 0) -> dict:
    frame = build_mock_frame(seq, elapsed)
    wave = np.asarray(
        [
            0.025 * math.sin(elapsed * 0.7),
            0.020 * math.sin(elapsed * 0.5 + 0.4),
            0.020 * math.sin(elapsed * 0.9 + 0.8),
        ]
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        hand = frame["hands"][side]
        for index, position in enumerate(hand["positions"]):
            base = np.asarray(position, dtype=float)
            hand["positions"][index] = (base + wave * np.asarray([1.0, sign, 1.0])).tolist()
    frame["xr_session_id"] = "m6-dryrun-session"
    frame["session"]["reference_space_revision"] = int(revision)
    return frame


def run_mapping_replay(arm_config: dict) -> dict:
    mapping = arm_config.get("mapping", {})
    axes = np.asarray(mapping.get("robot_from_vr_axes", np.eye(3)), dtype=float)
    scale = np.asarray(mapping.get("position_scale_xyz", [1, 1, 1]), dtype=float)
    neutral_hand = ArmTarget([0.2, 1.2, -0.3], [0, 0, 0, 1])
    moved_hand = ArmTarget([0.23, 1.18, -0.26], [0, 0, 0, 1])
    robot_anchor = ArmTarget([0.45, 0.35, 1.05], [0, 0, 0, 1])
    expected_delta = scale * (axes @ (moved_hand.position_array() - neutral_hand.position_array()))
    expected_position = robot_anchor.position_array() + expected_delta

    relative = ArmMapper(
        position_scale_xyz=scale,
        rotation_scale=float(mapping.get("rotation_scale", 1.0)),
        robot_from_vr_axes=axes,
        enable_orientation=bool(mapping.get("enable_orientation", False)),
        mapping_mode="relative",
    )
    relative.engage("left", neutral_hand, robot_anchor)
    relative_target = relative.map_hand("left", moved_hand)

    samples = [
        AbsoluteCalibrationSample(
            receive_time_ns=index * 10_000_000,
            xr_session_id="mapping-replay",
            reference_space="local-floor",
            reference_space_revision=0,
            hmd=PoseSample((0.0, 1.6, 0.0), (0, 0, 0, 1)),
            hands={"left": PoseSample(tuple(neutral_hand.position), tuple(neutral_hand.orientation_xyzw))},
            robot_desired={"left": PoseSample(tuple(robot_anchor.position), tuple(robot_anchor.orientation_xyzw))},
            robot_current={"left": PoseSample(tuple(robot_anchor.position), tuple(robot_anchor.orientation_xyzw))},
        )
        for index in range(4)
    ]
    absolute_result = build_absolute_calibration(
        samples,
        ["left"],
        axes,
        AbsoluteCalibrationConfig(countdown_sec=0, sample_duration_sec=0, minimum_valid_samples=4),
    )
    absolute = ArmMapper(
        position_scale_xyz=scale,
        rotation_scale=float(mapping.get("rotation_scale", 1.0)),
        robot_from_vr_axes=axes,
        enable_orientation=bool(mapping.get("enable_orientation", False)),
        mapping_mode="absolute",
    )
    absolute.set_absolute_calibration(absolute_result)
    absolute_target = absolute.map_hand(
        "left",
        moved_hand,
        xr_session_id="mapping-replay",
        reference_space="local-floor",
        reference_space_revision=0,
    )
    relative_error = float(np.linalg.norm(relative_target.position_array() - expected_position))
    absolute_error = float(np.linalg.norm(absolute_target.position_array() - expected_position))
    return {
        "same_input_delta_vr_m": (moved_hand.position_array() - neutral_hand.position_array()).tolist(),
        "expected_target_position_m": expected_position.tolist(),
        "relative_target_position_m": relative_target.position_array().tolist(),
        "absolute_target_position_m": absolute_target.position_array().tolist(),
        "relative_math_error_m": relative_error,
        "absolute_math_error_m": absolute_error,
        "passed": relative_error <= 1e-9 and absolute_error <= 1e-9,
    }


def augment_report_with_mapping_replay(arm_config: dict, output_dir: str | Path) -> dict:
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    replay = run_mapping_replay(arm_config)
    replay_path = output / "mapping_replay.yaml"
    replay_path.write_text(yaml.safe_dump(replay, sort_keys=False), encoding="utf-8")
    report_path = output / "report.yaml"
    if report_path.exists():
        report = yaml.safe_load(report_path.read_text(encoding="utf-8")) or {}
        telemetry_path = output / "telemetry.csv"
        telemetry_rows = list(csv.DictReader(telemetry_path.open(encoding="utf-8"))) if telemetry_path.exists() else []
        scenario_checks = analyze_scenario_rows(telemetry_rows)
        report["same_trajectory_mapping_replay"] = replay
        report["scenario_checks"] = scenario_checks
        report.setdefault("artifacts", {})["mapping_replay"] = str(replay_path)
        violations = list(report.get("violations", []))
        violations = [item for item in violations if item != "same-trajectory mapping replay failed"]
        if not replay["passed"]:
            violations.append("same-trajectory mapping replay failed")
        if telemetry_rows and not all(scenario_checks.values()):
            violations.append("one or more injected scenario safety behaviors were not observed")
        report["violations"] = violations
        report["result"] = "PASS" if not violations else "FAIL"
        report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    return replay


def analyze_scenario_rows(rows: list[dict]) -> dict[str, bool]:
    tracking_rows = [row for row in rows if row.get("scenario") == "tracking_lost"]
    disconnect_rows = [row for row in rows if row.get("scenario") == "disconnect"]
    delay_20_rows = [row for row in rows if row.get("scenario") == "sdk_delay_20ms"]

    def unique_positions(selected, side: str) -> set[tuple[str, str, str]]:
        return {(row[f"{side}_x"], row[f"{side}_y"], row[f"{side}_z"]) for row in selected}

    left_positions = unique_positions(tracking_rows, "left")
    right_positions = unique_positions(tracking_rows, "right")
    return {
        "tracking_lost_left_held": bool(tracking_rows) and len(left_positions) == 1,
        "tracking_lost_right_continued": len(right_positions) > 1,
        "disconnect_entered_paused": any(row.get("loop_state") == "PAUSED" for row in disconnect_rows),
        "sdk_20ms_entered_fault": any(row.get("loop_state") == "FAULT" for row in delay_20_rows),
    }


class DryRunValidation:
    def __init__(self, arm_config: dict, output_dir: str | Path, duration_sec: float = 600.0, input_rate_hz: float = 60.0):
        self.arm_config = arm_config
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.duration_sec = float(duration_sec)
        self.input_rate_hz = float(input_rate_hz)
        if self.duration_sec <= 1.0 or self.input_rate_hz <= 0.0:
            raise ValueError("duration_sec must exceed 1 second and input_rate_hz must be positive")
        self.adapter = DelayInjectingDryRunAdapter(
            freq_hz=float(arm_config.get("control_rate_hz", 100.0)),
            enable_real=False,
        )
        self.supervisor = ControlPCSupervisor(arm_config, arm="both", adapter=self.adapter)
        self.rows: list[TelemetryRow] = []
        self.events: list[dict] = []
        self._previous_targets: dict[str, tuple[float, ArmTarget]] = {}
        self._workspace_clips = 0
        self._active_samples = 0
        self._active_elapsed_sec = 0.0
        self._active_sent_cycles = 0
        self._nominal_active_elapsed_sec = 0.0
        self._nominal_active_sent_cycles = 0
        self._previous_sample: tuple[float, str, int] | None = None
        self.mapping_checks: dict[str, dict] = {}

    def run(self) -> dict:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        start = time.monotonic()
        next_frame = start
        next_sample = start
        seq = 0
        revision = 0
        scenario = "nominal"
        last_phase = ""
        self.supervisor.start()
        try:
            self._verify_mapping_modes(start)
            while True:
                now = time.monotonic()
                elapsed = now - start
                if elapsed >= self.duration_sec:
                    break
                phase = self._phase(elapsed)
                if phase != last_phase:
                    scenario, revision = self._enter_phase(phase, elapsed, seq, revision)
                    last_phase = phase
                if now >= next_frame and phase != "disconnect":
                    frame = build_validation_frame(seq, elapsed, revision)
                    if phase == "tracking_lost":
                        frame["hands"]["left"] = {
                            "valid": False,
                            "joint_names": [],
                            "positions": [],
                            "orientations_xyzw": [],
                        }
                    self.supervisor.ingest_payload(frame)
                    seq += 1
                    next_frame += 1.0 / self.input_rate_hz
                    if next_frame < now - 0.1:
                        next_frame = now
                if now >= next_sample:
                    self._sample(elapsed, seq, scenario)
                    next_sample += 0.05
                time.sleep(0.001)
        finally:
            self.adapter.set_delay(0.0)
            self.supervisor.close()
        return self._write_outputs(start, time.monotonic(), seq)

    def _verify_mapping_modes(self, start: float) -> None:
        frame = build_validation_frame(0, 0.0, 0)
        self.supervisor.ingest_payload(frame)
        time.sleep(0.02)
        recenter = self.supervisor.execute_command("recenter")
        desired = self.adapter.get_desired_poses()
        relative_zero = self.supervisor._candidate_targets(self.supervisor.tracking_buffer.snapshot().frame)
        self.mapping_checks["relative"] = {
            "prepared": recenter.accepted,
            "zero_input_position_error_m": max(
                float(np.linalg.norm(relative_zero[side].position_array() - desired[side].position_array()))
                for side in self.supervisor.enabled_sides
            ),
        }
        self.supervisor.execute_command("pause")
        calibration_config = self.supervisor.calibrator.config
        original_countdown = calibration_config.countdown_sec
        original_duration = calibration_config.sample_duration_sec
        original_minimum = calibration_config.minimum_valid_samples
        object.__setattr__(calibration_config, "countdown_sec", 0.0)
        object.__setattr__(calibration_config, "sample_duration_sec", 0.06)
        object.__setattr__(calibration_config, "minimum_valid_samples", 4)
        try:
            started = self.supervisor.execute_command("absolute-calibrate")
            neutral = build_validation_frame(0, 0.0, 0)
            for seq in range(1, 9):
                neutral["seq"] = seq
                self.supervisor.ingest_payload(neutral)
                time.sleep(0.012)
            deadline = time.monotonic() + 1.0
            while self.supervisor.calibrator.state not in {CalibrationState.VALID, CalibrationState.INVALID} and time.monotonic() < deadline:
                time.sleep(0.01)
            status = self.supervisor.status_snapshot()
            absolute_error = None
            if status.calibration_state == "VALID" and status.mapping_mode == "absolute":
                current_frame = self.supervisor.tracking_buffer.snapshot().frame
                absolute = self.supervisor._candidate_targets(current_frame)
                desired = self.adapter.get_desired_poses()
                absolute_error = max(
                    float(np.linalg.norm(absolute[side].position_array() - desired[side].position_array()))
                    for side in self.supervisor.enabled_sides
                )
            self.mapping_checks["absolute"] = {
                "calibration_started": started.accepted,
                "calibration_state": status.calibration_state,
                "activation_state": status.teleop_state,
                "first_target_position_error_m": absolute_error,
                "sent_calls_during_calibration": self.adapter.stats.send_calls,
            }
        finally:
            object.__setattr__(calibration_config, "countdown_sec", original_countdown)
            object.__setattr__(calibration_config, "sample_duration_sec", original_duration)
            object.__setattr__(calibration_config, "minimum_valid_samples", original_minimum)
        self.supervisor.execute_command("mode", "relative")
        self._recover_relative(20, time.monotonic() - start, 0)

    def _phase(self, elapsed: float) -> str:
        fraction = elapsed / self.duration_sec
        if 0.12 <= fraction < 0.14:
            return "tracking_lost"
        if 0.28 <= fraction < 0.30:
            return "disconnect"
        if 0.44 <= fraction < 0.46:
            return "session_reset"
        if 0.60 <= fraction < 0.66:
            return "sdk_delay_5ms"
        if 0.70 <= fraction < 0.76:
            return "sdk_delay_10ms"
        if 0.80 <= fraction:
            return "sdk_delay_20ms"
        return "nominal"

    def _enter_phase(self, phase: str, elapsed: float, seq: int, revision: int) -> tuple[str, int]:
        self.adapter.set_delay(0.0)
        if phase == "nominal":
            self._recover_relative(seq, elapsed, revision)
        elif phase == "session_reset":
            revision += 1
            self.supervisor.submit_command("invalidate-calibration", "injected reference-space reset")
        elif phase.startswith("sdk_delay_"):
            delay_ms = float(phase.removeprefix("sdk_delay_").removesuffix("ms"))
            self.adapter.set_delay(delay_ms / 1000.0)
            self._recover_relative(seq, elapsed, revision)
        self.events.append({"elapsed_sec": elapsed, "event": phase, "revision": revision})
        return phase, revision

    def _recover_relative(self, seq: int, elapsed: float, revision: int) -> None:
        status = self.supervisor.status_snapshot()
        if status.loop_state == "FAULT":
            return
        self.supervisor.execute_command("pause")
        frame = build_validation_frame(seq, elapsed, revision)
        self.supervisor.ingest_payload(frame)
        time.sleep(0.015)
        status = self.supervisor.status_snapshot()
        if status.teleop_state == "ESTOP" or status.loop_state == "FAULT":
            return
        if status.mapping_mode != "relative":
            self.supervisor.execute_command("mode", "relative")
        recenter = self.supervisor.execute_command("recenter")
        if recenter.accepted:
            self.adapter.reset_velocity_history()
            self.supervisor.execute_command("start")
            self._previous_targets.clear()
            self._previous_sample = None

    def _sample(self, elapsed: float, seq: int, scenario: str) -> None:
        status = self.supervisor.status_snapshot()
        snapshot = self.supervisor.tracking_buffer.snapshot()
        age_ms = 0.0 if snapshot is None else max(0.0, (time.monotonic_ns() - snapshot.receive_time_ns) / 1e6)
        values = {}
        for side in ("left", "right"):
            target = self.adapter.last_targets.get(side)
            position = np.zeros(3) if target is None else target.position_array()
            speed = 0.0
            angular_speed = 0.0
            previous = self._previous_targets.get(side)
            if target is not None and previous is not None:
                previous_time, previous_target = previous
                dt = max(1e-9, elapsed - previous_time)
                speed = float(np.linalg.norm(position - previous_target.position_array()) / dt)
                angular_speed = quat_angle_xyzw(target.orientation_array(), previous_target.orientation_array()) / dt
            if target is not None:
                self._previous_targets[side] = (elapsed, target)
                arm_filter = self.supervisor.arm_filters[side]
                if np.any(position <= arm_filter.xyz_min + 1e-9) or np.any(position >= arm_filter.xyz_max - 1e-9):
                    self._workspace_clips += 1
            values[side] = (*position.tolist(), speed, angular_speed)
        self.rows.append(
            TelemetryRow(
                elapsed, seq, scenario, status.teleop_state, status.loop_state, age_ms,
                status.sent_cycles, status.missed_deadlines,
                *values["left"], *values["right"],
            )
        )
        if status.loop_state == "ACTIVE":
            self._active_samples += 1
        if self._previous_sample is not None:
            previous_time, previous_state, previous_sent = self._previous_sample
            if previous_state == "ACTIVE" and status.loop_state == "ACTIVE":
                self._active_elapsed_sec += elapsed - previous_time
                self._active_sent_cycles += max(0, status.sent_cycles - previous_sent)
                if scenario == "nominal":
                    self._nominal_active_elapsed_sec += elapsed - previous_time
                    self._nominal_active_sent_cycles += max(0, status.sent_cycles - previous_sent)
        self._previous_sample = (elapsed, status.loop_state, status.sent_cycles)

    def _write_outputs(self, started: float, ended: float, published_frames: int) -> dict:
        csv_path = self.output_dir / "telemetry.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(TelemetryRow.__dataclass_fields__))
            writer.writeheader()
            writer.writerows(asdict(row) for row in self.rows)
        self._write_svg(self.output_dir / "telemetry.svg")
        sampled_max_linear = max((max(row.left_speed_mps, row.right_speed_mps) for row in self.rows), default=0.0)
        sampled_max_angular = max((max(row.left_angular_speed_rad_s, row.right_angular_speed_rad_s) for row in self.rows), default=0.0)
        max_linear = max(self.supervisor.loop.stats.target_linear_speeds_mps, default=0.0)
        max_angular = max(self.supervisor.loop.stats.target_angular_speeds_rad_s, default=0.0)
        finite = all(np.isfinite(value) for row in self.rows for value in (
            row.left_x, row.left_y, row.left_z, row.left_speed_mps, row.left_angular_speed_rad_s,
            row.right_x, row.right_y, row.right_z, row.right_speed_mps, row.right_angular_speed_rad_s,
        ))
        safety = self.arm_config.get("safety", {})
        report = {
            "result": "PASS",
            "dry_run_only": True,
            "real_hardware_initialized": False,
            "requested_duration_sec": self.duration_sec,
            "actual_duration_sec": ended - started,
            "input_rate_hz": self.input_rate_hz,
            "published_frames": published_frames,
            "consumed_frames": self.supervisor.loop.stats.consumed_frames,
            "sent_cycles": self.supervisor.loop.stats.sent_cycles,
            "output_rate_hz": self.supervisor.loop.stats.sent_cycles / max(1e-9, ended - started),
            "active_output_rate_hz": (
                0.0 if self._active_elapsed_sec <= 0.0 else self._active_sent_cycles / self._active_elapsed_sec
            ),
            "nominal_active_output_rate_hz": (
                0.0
                if self._nominal_active_elapsed_sec <= 0.0
                else self._nominal_active_sent_cycles / self._nominal_active_elapsed_sec
            ),
            "latest_buffer_size": self.supervisor.tracking_buffer.size,
            "finite_targets": finite,
            "max_linear_speed_mps": max_linear,
            "sampled_max_linear_speed_mps": sampled_max_linear,
            "sdk_wallclock_max_linear_speed_mps": max(self.adapter.command_linear_speeds, default=0.0),
            "configured_max_linear_speed_mps": float(safety.get("max_linear_speed_mps", 0.1)),
            "max_angular_speed_rad_s": max_angular,
            "sampled_max_angular_speed_rad_s": sampled_max_angular,
            "sdk_wallclock_max_angular_speed_rad_s": max(self.adapter.command_angular_speeds, default=0.0),
            "configured_max_angular_speed_rad_s": float(safety.get("max_angular_speed_rad_s", 0.5)),
            "workspace_boundary_samples": self._workspace_clips,
            "stale_pauses": self.supervisor.loop.stats.stale_pauses,
            "missed_deadlines": self.supervisor.loop.stats.missed_deadlines,
            "loop_period_ms": {
                "p50": percentile_ms(self.supervisor.loop.stats.loop_period_ns, 50),
                "p95": percentile_ms(self.supervisor.loop.stats.loop_period_ns, 95),
                "p99": percentile_ms(self.supervisor.loop.stats.loop_period_ns, 99),
            },
            "sdk_call_ms": {
                "p50": percentile_ms(self.supervisor.loop.stats.sdk_call_time_ns, 50),
                "p95": percentile_ms(self.supervisor.loop.stats.sdk_call_time_ns, 95),
                "p99": percentile_ms(self.supervisor.loop.stats.sdk_call_time_ns, 99),
            },
            "injected_sdk_delay": {
                f"{delay_ms}ms": {
                    "calls": self.adapter.delay_call_counts.get(delay_ms, 0),
                    "observed_min_ms": min(self.adapter.delay_observed_ms.get(delay_ms, [0.0])),
                    "observed_max_ms": max(self.adapter.delay_observed_ms.get(delay_ms, [0.0])),
                }
                for delay_ms in (5, 10, 20)
            },
            "events": self.events,
            "mapping_checks": self.mapping_checks,
            "same_trajectory_mapping_replay": run_mapping_replay(self.arm_config),
            "scenario_checks": analyze_scenario_rows([asdict(row) for row in self.rows]),
            "artifacts": {"telemetry_csv": str(csv_path), "telemetry_svg": str(self.output_dir / "telemetry.svg")},
        }
        violations = []
        if not finite:
            violations.append("non-finite target telemetry")
        if max_linear > float(safety.get("max_linear_speed_mps", 0.1)) * 1.10:
            violations.append("linear speed exceeded configured limit")
        if max_angular > float(safety.get("max_angular_speed_rad_s", 0.5)) * 1.10:
            violations.append("angular speed exceeded configured limit")
        if self.supervisor.tracking_buffer.size != 1:
            violations.append("latest-value buffer grew beyond one frame")
        nominal_rate = report["nominal_active_output_rate_hz"]
        if nominal_rate < 90.0 or nominal_rate > 110.0:
            violations.append(f"nominal ACTIVE output rate {nominal_rate:.2f} Hz is outside 90-110 Hz")
        required_events = {"tracking_lost", "disconnect", "session_reset", "sdk_delay_5ms", "sdk_delay_10ms", "sdk_delay_20ms"}
        observed_events = {event["event"] for event in self.events}
        if not required_events.issubset(observed_events):
            violations.append("not all fault-injection scenarios executed")
        for delay_ms in (5, 10, 20):
            if self.adapter.delay_call_counts.get(delay_ms, 0) == 0:
                violations.append(f"{delay_ms} ms SDK delay was not exercised by an output call")
        if self.mapping_checks.get("relative", {}).get("zero_input_position_error_m", 1.0) > 1e-9:
            violations.append("relative zero-input target did not equal desired pose")
        absolute = self.mapping_checks.get("absolute", {})
        if absolute.get("calibration_state") != "VALID" or absolute.get("activation_state") != "ARMED":
            violations.append("absolute calibration did not finish VALID/ARMED")
        if absolute.get("first_target_position_error_m") is None or absolute["first_target_position_error_m"] > 1e-6:
            violations.append("absolute first target did not equal calibration anchor")
        if absolute.get("sent_calls_during_calibration") != 0:
            violations.append("absolute calibration sent an arm command")
        if not report["same_trajectory_mapping_replay"]["passed"]:
            violations.append("same-trajectory mapping replay failed")
        if not all(report["scenario_checks"].values()):
            violations.append("one or more injected scenario safety behaviors were not observed")
        report["violations"] = violations
        report["result"] = "PASS" if not violations else "FAIL"
        report_path = self.output_dir / "report.yaml"
        report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        return report

    def _write_svg(self, path: Path) -> None:
        width, height, margin = 1200, 500, 45
        if not self.rows:
            path.write_text("<svg xmlns='http://www.w3.org/2000/svg'/>", encoding="utf-8")
            return
        max_time = max(row.elapsed_sec for row in self.rows) or 1.0
        max_value = max(0.1, max(max(row.left_speed_mps, row.right_speed_mps) for row in self.rows))
        points = []
        for row in self.rows:
            x = margin + row.elapsed_sec / max_time * (width - 2 * margin)
            y = height - margin - max(row.left_speed_mps, row.right_speed_mps) / max_value * (height - 2 * margin)
            points.append(f"{x:.2f},{y:.2f}")
        svg = (
            f"<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}'>"
            "<rect width='100%' height='100%' fill='white'/>"
            f"<line x1='{margin}' y1='{height-margin}' x2='{width-margin}' y2='{height-margin}' stroke='black'/>"
            f"<line x1='{margin}' y1='{margin}' x2='{margin}' y2='{height-margin}' stroke='black'/>"
            f"<polyline fill='none' stroke='#1565c0' stroke-width='1.5' points='{' '.join(points)}'/>"
            f"<text x='{width/2}' y='24' text-anchor='middle'>Dry-run arm target linear speed (m/s)</text>"
            f"<text x='{width/2}' y='{height-8}' text-anchor='middle'>elapsed seconds</text>"
            "</svg>"
        )
        path.write_text(svg, encoding="utf-8")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run M6 mock-only arm teleoperation validation.")
    parser.add_argument("--config", default="configs/arm/s1_quest3_default.yaml")
    parser.add_argument("--duration-sec", type=float, default=600.0)
    parser.add_argument("--input-rate-hz", type=float, default=60.0)
    parser.add_argument("--output-dir", default="logs/m6_dryrun_validation")
    parser.add_argument("--mapping-replay-only", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    arm_config = load_yaml_config(args.config)
    if args.mapping_replay_only:
        replay = augment_report_with_mapping_replay(arm_config, args.output_dir)
        print(yaml.safe_dump(replay, sort_keys=False), flush=True)
        return 0 if replay["passed"] else 1
    validation = DryRunValidation(arm_config, args.output_dir, args.duration_sec, args.input_rate_hz)
    report = validation.run()
    print(yaml.safe_dump(report, sort_keys=False), flush=True)
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

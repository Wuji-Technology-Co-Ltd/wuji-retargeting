from __future__ import annotations

import hashlib
import importlib.util
import math
import os
import signal
import subprocess
import builtins
import sys
import types
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
from importlib import import_module
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any, Callable, Mapping

import numpy as np
import yaml


SCHEMA = "astribot_s1_m7_hardware_audit.v1"
REQUIRED_SCENARIOS = (
    "sdk_read_only",
    "static_hold",
    "normal_exit",
    "ctrl_c",
    "process_kill",
    "network_disconnect",
)
SCENARIO_REQUIRED_ACTION_GROUPS = {
    "sdk_read_only": ({"operator_confirmed_before_sdk_initialization"}, {"live_sdk_read_only_snapshot"}),
    "static_hold": (
        {"operator_confirmed_before_sdk_initialization"},
        {"pre_static_hold_snapshot"},
        {"exact_desired_static_hold"},
    ),
    "normal_exit": ({"sdk_shutdown_returned"},),
    "ctrl_c": (
        {"operator_confirmed_before_sdk_initialization"},
        {"failure_monitor_initial_snapshot"},
        {"failure_monitor_started"},
        {"keyboard_interrupt_received"},
        {"sdk_shutdown_returned_after_monitor", "sdk_shutdown_failed"},
    ),
    "process_kill": (
        {"operator_confirmed_before_sdk_initialization"},
        {"failure_monitor_initial_snapshot"},
        {"failure_monitor_started"},
        {"manual_process_kill_verified"},
    ),
    "network_disconnect": (
        {"operator_confirmed_before_sdk_initialization"},
        {"failure_monitor_initial_snapshot"},
        {"failure_monitor_started"},
        {"physical_network_disconnect_verified"},
        {
            "sdk_shutdown_returned_after_monitor",
            "sdk_shutdown_failed",
            "monitor_process_absent_after_disconnect",
        },
    ),
}
SCENARIO_REQUIRED_CAPTURE_PHASES = {
    "sdk_read_only": {"during"},
    "static_hold": {"after"},
    "normal_exit": {"after"},
    "ctrl_c": {"before", "after"},
    "process_kill": {"before", "after"},
    "network_disconnect": {"before", "after"},
}
SCENARIO_CAPTURE_TIME_ANCHORS = {
    "sdk_read_only": {"during": ("after", {"live_sdk_read_only_snapshot"})},
    "static_hold": {"after": ("after", {"exact_desired_static_hold"})},
    "normal_exit": {"after": ("after", {"sdk_shutdown_returned"})},
    "ctrl_c": {
        "before": ("before", {"failure_monitor_started"}),
        "after": ("after", {"keyboard_interrupt_received"}),
    },
    "process_kill": {
        "before": ("before", {"failure_monitor_started"}),
        "after": ("after", {"manual_process_kill_verified"}),
    },
    "network_disconnect": {
        "before": ("before", {"failure_monitor_started"}),
        "after": ("after", {"physical_network_disconnect_verified"}),
    },
}
LIVE_CONFIRMATION = "M7 READY PHYSICAL ESTOP"
HOLD_CONFIRMATION = "SEND EXACT DESIRED HOLD"
OBSERVATION_CONFIRMATION = "M7 OBSERVATION VERIFIED"
PROCESS_KILL_CONFIRMATION = "M7 READY MANUAL SIGKILL"
NETWORK_DISCONNECT_CONFIRMATION = "M7 READY PHYSICAL NETWORK DISCONNECT"
PROCESS_KILL_VERIFIED_CONFIRMATION = "M7 SIGKILL PERFORMED AND OBSERVED"
NETWORK_DISCONNECT_VERIFIED_CONFIRMATION = "M7 NETWORK DISCONNECT PERFORMED AND OBSERVED"
NETWORK_MONITOR_ABSENT_CONFIRMATION = "M7 MONITOR PROCESS ABSENT AFTER DISCONNECT"
ENDPOINT_TOPICS = (
    "/astribot_arm_left/endpoint_current_states",
    "/astribot_arm_right/endpoint_current_states",
    "/astribot_arm_left/endpoint_desired_states",
    "/astribot_arm_right/endpoint_desired_states",
)
MAX_STATIC_HOLD_DESIRED_CURRENT_ERROR_M = 0.02


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _process_exists(pid: int) -> bool:
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _process_start_ticks(pid: int) -> int | None:
    try:
        stat = Path(f"/proc/{int(pid)}/stat").read_text(encoding="utf-8")
        remainder = stat.rsplit(") ", 1)[1].split()
        return int(remainder[19])
    except (FileNotFoundError, IndexError, PermissionError, ValueError):
        return None


def prepare_vendor_robotics_import(sdk_root: str | Path) -> None:
    package_dir = (
        Path(sdk_root).expanduser().resolve()
        / "astribot_sdk/core/common/robotics_library_py"
    )
    native_paths = {
        "robotics_library_py": package_dir / "robotics_library_py.so",
        "robotics_library_base": package_dir / "robotics_library_base.so",
    }
    missing = [str(path) for path in native_paths.values() if not path.is_file()]
    if missing:
        raise RuntimeError("Vendor robotics native modules are missing: " + ", ".join(missing))

    existing = sys.modules.get("robotics_library_py")
    if existing is not None:
        if getattr(existing, "__m7_vendor_shim__", False):
            return
        raise RuntimeError(
            "robotics_library_py was imported before the M7 Vendor shim; start a fresh process"
        )

    package = types.ModuleType("robotics_library_py")
    package.__file__ = str(package_dir / "__init__.py")
    package.__package__ = "robotics_library_py"
    package.__path__ = [str(package_dir)]
    package.__m7_vendor_shim__ = True
    aliases = (
        "robotics_library_py",
        "astribot_sdk.core.common.robotics_library_py",
    )
    for alias in aliases:
        sys.modules[alias] = package

    loaded_names = list(aliases)
    try:
        for short_name, path in native_paths.items():
            canonical_name = f"robotics_library_py.{short_name}"
            spec = importlib.util.spec_from_file_location(canonical_name, path)
            if spec is None or spec.loader is None:
                raise RuntimeError(f"cannot create import spec for {path}")
            module = importlib.util.module_from_spec(spec)
            sys.modules[canonical_name] = module
            loaded_names.append(canonical_name)
            spec.loader.exec_module(module)
            full_name = f"astribot_sdk.core.common.robotics_library_py.{short_name}"
            sys.modules[full_name] = module
            loaded_names.append(full_name)
            for name, value in vars(module).items():
                if not name.startswith("__"):
                    setattr(package, name, value)
    except Exception:
        for name in reversed(loaded_names):
            sys.modules.pop(name, None)
        raise


def require_confirmation(prompt: str, phrase: str, input_fn: Callable[[str], str] = input) -> str:
    answer = input_fn(f"{prompt}\nType exactly: {phrase}\n> ").strip()
    if answer != phrase:
        raise RuntimeError("confirmation phrase did not match; audit stopped fail-closed")
    return answer


def _new_report() -> dict[str, Any]:
    now = utc_now()
    return {
        "schema": SCHEMA,
        "status": "INCOMPLETE",
        "created_at": now,
        "updated_at": now,
        "safety": {
            "quest_dynamic_input_connected": False,
            "relative_absolute_teleop_enabled": False,
            "internal_topic_publishers_allowed": False,
            "moving_or_home_commands_allowed": False,
        },
        "environment": {},
        "static_sdk_findings": {},
        "scenarios": {
            name: {"status": "PENDING", "disposition": "unknown", "observation": "", "steps": []}
            for name in REQUIRED_SCENARIOS
        },
        "m8_permitted": False,
        "ros_captures": [],
        "steps": [],
        "completion_reasons": [f"{name} is PENDING" for name in REQUIRED_SCENARIOS],
    }


class M7ReportStore:
    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser().resolve()

    def load(self) -> dict[str, Any]:
        if not self.path.exists():
            return _new_report()
        data = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
        if data.get("schema") != SCHEMA:
            raise ValueError(f"unsupported M7 report schema in {self.path}")
        self._update_completion(data)
        return data

    def save(self, report: Mapping[str, Any]) -> None:
        data = deepcopy(dict(report))
        data["updated_at"] = utc_now()
        self._update_completion(data)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with NamedTemporaryFile("w", encoding="utf-8", dir=self.path.parent, delete=False) as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
            temporary = Path(stream.name)
        temporary.replace(self.path)

    def initialize(self, static_findings: Mapping[str, Any] | None = None) -> dict[str, Any]:
        report = self.load()
        report["environment"] = {
            "ROS_DOMAIN_ID": os.getenv("ROS_DOMAIN_ID"),
            "RMW_IMPLEMENTATION": os.getenv("RMW_IMPLEMENTATION"),
            "ROS_DISTRO": os.getenv("ROS_DISTRO"),
            "ROBOT_TYPE": os.getenv("ROBOT_TYPE"),
        }
        if static_findings is not None:
            report["static_sdk_findings"] = deepcopy(dict(static_findings))
        self.save(report)
        return self.load()

    def append_step(self, scenario: str, step: Mapping[str, Any]) -> dict[str, Any]:
        if scenario not in REQUIRED_SCENARIOS:
            raise ValueError(f"unsupported M7 scenario: {scenario}")
        report = self.load()
        entry = {"timestamp": utc_now(), **deepcopy(dict(step))}
        report["steps"].append(entry)
        report["scenarios"][scenario]["steps"].append(entry)
        self.save(report)
        return entry

    def add_ros_capture(self, scenario: str, phase: str, capture: Mapping[str, Any], confirmation: str) -> dict[str, Any]:
        if scenario not in REQUIRED_SCENARIOS:
            raise ValueError(f"unsupported M7 scenario: {scenario}")
        if phase not in {"before", "during", "after"}:
            raise ValueError(f"unsupported ROS capture phase: {phase}")
        if confirmation != OBSERVATION_CONFIRMATION:
            raise RuntimeError("ROS capture confirmation phrase did not match")
        report = self.load()
        entry = {
            "timestamp": utc_now(),
            "scenario": scenario,
            "phase": phase,
            "operator_confirmation": confirmation,
            "capture": deepcopy(dict(capture)),
        }
        report["ros_captures"].append(entry)
        report["steps"].append({"action": "read_only_ros_capture", **entry})
        if scenario in report["scenarios"]:
            report["scenarios"][scenario]["steps"].append({"action": "read_only_ros_capture", **entry})
        self.save(report)
        return entry

    def complete_scenario(self, scenario: str, observation: str, confirmation: str, disposition: str) -> None:
        if scenario not in REQUIRED_SCENARIOS:
            raise ValueError(f"unsupported M7 scenario: {scenario}")
        if not observation.strip():
            raise ValueError("a non-empty onsite observation is required")
        if disposition not in {"safe", "unsafe", "unknown"}:
            raise ValueError("disposition must be safe, unsafe, or unknown")
        if confirmation != OBSERVATION_CONFIRMATION:
            raise RuntimeError("onsite observation confirmation phrase did not match")
        report = self.load()
        if report.get("scenarios", {}).get(scenario, {}).get("status") == "COMPLETE":
            raise RuntimeError(
                f"{scenario} is already COMPLETE; onsite disposition is immutable in this report"
            )
        actions = {step.get("action") for step in report["scenarios"][scenario].get("steps", [])}
        missing_groups = [group for group in SCENARIO_REQUIRED_ACTION_GROUPS[scenario] if not actions.intersection(group)]
        if missing_groups:
            missing = "; ".join("one of " + ", ".join(sorted(group)) for group in missing_groups)
            raise RuntimeError(f"{scenario} is missing required audit evidence: {missing}")
        valid_phases = {
            item.get("phase")
            for item in report.get("ros_captures", [])
            if item.get("scenario") == scenario
            and ros_capture_evidence_valid(item.get("capture", {}))
            and item.get("operator_confirmation") == OBSERVATION_CONFIRMATION
        }
        missing_phases = SCENARIO_REQUIRED_CAPTURE_PHASES[scenario] - valid_phases
        if missing_phases:
            missing = ", ".join(sorted(missing_phases))
            raise RuntimeError(f"{scenario} is missing valid required read-only ROS captures: {missing}")
        timing_reasons = self._capture_timing_reasons(report, scenario)
        if timing_reasons:
            raise RuntimeError("; ".join(timing_reasons))
        runtime_reasons = self._runtime_evidence_reasons(report, scenario)
        if runtime_reasons:
            raise RuntimeError("; ".join(runtime_reasons))
        report["scenarios"][scenario]["status"] = "COMPLETE"
        report["scenarios"][scenario]["disposition"] = disposition
        report["scenarios"][scenario]["observation"] = observation.strip()
        entry = {
            "timestamp": utc_now(),
            "action": "onsite_observation",
            "scenario": scenario,
            "observation": observation.strip(),
            "disposition": disposition,
            "operator_confirmation": confirmation,
        }
        report["steps"].append(entry)
        report["scenarios"][scenario]["steps"].append(entry)
        self.save(report)

    def record_failure_event(
        self,
        scenario: str,
        confirmation: str,
        process_exists_fn: Callable[[int], bool] | None = None,
        process_start_ticks_fn: Callable[[int], int | None] | None = None,
    ) -> dict[str, Any]:
        expected = {
            "process_kill": (PROCESS_KILL_VERIFIED_CONFIRMATION, "manual_process_kill_verified"),
            "network_disconnect": (
                NETWORK_DISCONNECT_VERIFIED_CONFIRMATION,
                "physical_network_disconnect_verified",
            ),
        }
        if scenario not in expected:
            raise ValueError("failure event scenario must be process_kill or network_disconnect")
        expected_confirmation, action = expected[scenario]
        if confirmation != expected_confirmation:
            raise RuntimeError("failure event confirmation phrase did not match")
        report = self.load()
        steps = report.get("scenarios", {}).get(scenario, {}).get("steps", [])
        monitor_steps = [step for step in steps if step.get("action") == "failure_monitor_started"]
        if not monitor_steps:
            raise RuntimeError(f"{scenario} monitor evidence is missing; failure event cannot be recorded")
        monitor_pid = monitor_steps[-1].get("pid")
        if not isinstance(monitor_pid, int) or monitor_pid <= 0:
            raise RuntimeError(f"{scenario} monitor PID evidence is invalid")
        process_absent = None
        if scenario == "process_kill":
            if any(step.get("action") == "keyboard_interrupt_received" for step in steps):
                raise RuntimeError("process_kill cannot be verified after a recorded Ctrl+C shutdown")
            checker = process_exists_fn or _process_exists
            start_checker = process_start_ticks_fn or _process_start_ticks
            recorded_ticks = monitor_steps[-1].get("process_start_ticks")
            current_ticks = start_checker(monitor_pid)
            if checker(monitor_pid) and (recorded_ticks is None or current_ticks == recorded_ticks):
                raise RuntimeError(f"process_kill monitor process instance {monitor_pid} is still running")
            process_absent = True
        return self.append_step(
            scenario,
            {
                "action": action,
                "operator_confirmation": confirmation,
                "checked_monitor_pid": monitor_pid,
                "checked_process_start_ticks": recorded_ticks if scenario == "process_kill" else None,
                "observed_process_start_ticks": current_ticks if scenario == "process_kill" else None,
                "process_absent": process_absent,
            },
        )

    def record_network_monitor_absence(
        self,
        confirmation: str,
        process_exists_fn: Callable[[int], bool] | None = None,
        process_start_ticks_fn: Callable[[int], int | None] | None = None,
    ) -> dict[str, Any]:
        if confirmation != NETWORK_MONITOR_ABSENT_CONFIRMATION:
            raise RuntimeError("network monitor absence confirmation phrase did not match")
        report = self.load()
        steps = report.get("scenarios", {}).get("network_disconnect", {}).get("steps", [])
        monitor_steps = [step for step in steps if step.get("action") == "failure_monitor_started"]
        disconnect_steps = [step for step in steps if step.get("action") == "physical_network_disconnect_verified"]
        if not monitor_steps or not disconnect_steps:
            raise RuntimeError("network monitor and verified disconnect evidence are required")
        monitor_pid = monitor_steps[-1].get("pid")
        if not isinstance(monitor_pid, int) or monitor_pid <= 0:
            raise RuntimeError("network monitor PID evidence is invalid")
        checker = process_exists_fn or _process_exists
        start_checker = process_start_ticks_fn or _process_start_ticks
        recorded_ticks = monitor_steps[-1].get("process_start_ticks")
        current_ticks = start_checker(monitor_pid)
        if checker(monitor_pid) and (recorded_ticks is None or current_ticks == recorded_ticks):
            raise RuntimeError(f"network monitor process instance {monitor_pid} is still running")
        return self.append_step(
            "network_disconnect",
            {
                "action": "monitor_process_absent_after_disconnect",
                "operator_confirmation": confirmation,
                "checked_monitor_pid": monitor_pid,
                "checked_process_start_ticks": recorded_ticks,
                "observed_process_start_ticks": current_ticks,
                "process_absent": True,
            },
        )

    @staticmethod
    def _capture_timing_reasons(report: Mapping[str, Any], scenario: str) -> list[str]:
        scenario_data = report.get("scenarios", {}).get(scenario, {})
        steps = scenario_data.get("steps", [])
        captures = [
            item
            for item in report.get("ros_captures", [])
            if item.get("scenario") == scenario
            and ros_capture_evidence_valid(item.get("capture", {}))
            and item.get("operator_confirmation") == OBSERVATION_CONFIRMATION
        ]
        reasons = []
        for phase, (relation, action_names) in SCENARIO_CAPTURE_TIME_ANCHORS[scenario].items():
            action_times = [
                _parse_utc_timestamp(step.get("timestamp"))
                for step in steps
                if step.get("action") in action_names
            ]
            capture_times = [
                _parse_utc_timestamp(item.get("timestamp"))
                for item in captures
                if item.get("phase") == phase
            ]
            action_times = [value for value in action_times if value is not None]
            capture_times = [value for value in capture_times if value is not None]
            if not action_times or not capture_times:
                reasons.append(f"{scenario} {phase} capture lacks valid timestamp evidence")
                continue
            if relation == "before":
                ordered = any(capture_time <= action_time for capture_time in capture_times for action_time in action_times)
            else:
                ordered = any(capture_time >= action_time for capture_time in capture_times for action_time in action_times)
            if not ordered:
                reasons.append(f"{scenario} {phase} capture timestamp is not {relation} its runtime action")
        return reasons

    @staticmethod
    def _runtime_evidence_reasons(report: Mapping[str, Any], scenario: str) -> list[str]:
        steps = report.get("scenarios", {}).get(scenario, {}).get("steps", [])
        reasons = []

        def matching(action: str) -> list[Mapping[str, Any]]:
            return [step for step in steps if step.get("action") == action]

        if scenario in {"sdk_read_only", "static_hold", "ctrl_c", "process_kill", "network_disconnect"}:
            confirmations = matching("operator_confirmed_before_sdk_initialization")
            if not any(step.get("confirmation") == LIVE_CONFIRMATION for step in confirmations):
                reasons.append(f"{scenario} lacks exact live-SDK safety confirmation")

        snapshot_action = {
            "sdk_read_only": "live_sdk_read_only_snapshot",
            "static_hold": "pre_static_hold_snapshot",
            "ctrl_c": "failure_monitor_initial_snapshot",
            "process_kill": "failure_monitor_initial_snapshot",
            "network_disconnect": "failure_monitor_initial_snapshot",
        }.get(scenario)
        if snapshot_action:
            snapshots = [step.get("snapshot") for step in matching(snapshot_action)]
            if not any(not _snapshot_evidence_reasons(snapshot) for snapshot in snapshots):
                reasons.append(f"{scenario} lacks a structurally valid live SDK snapshot")
            if scenario in {"sdk_read_only", "static_hold"} and not any(
                isinstance(snapshot, Mapping)
                and not _snapshot_evidence_reasons(snapshot)
                and snapshot.get("robot_alive") is True
                and snapshot.get("robot_mode") == "safe"
                for snapshot in snapshots
            ):
                reasons.append(f"{scenario} lacks an alive safe-mode SDK snapshot")

        if scenario == "static_hold":
            holds = matching("exact_desired_static_hold")
            if not any(not _static_hold_evidence_reasons(step) for step in holds):
                reasons.append("static_hold lacks valid exact-desired single-command evidence")
        elif scenario == "normal_exit":
            shutdowns = [step for step in matching("sdk_shutdown_returned") if step.get("success") is True]
            if not shutdowns:
                reasons.append("normal_exit lacks successful SDK shutdown evidence")
            read_steps = report.get("scenarios", {}).get("sdk_read_only", {}).get("steps", [])
            live_confirmations = [
                step
                for step in read_steps
                if step.get("action") == "operator_confirmed_before_sdk_initialization"
                and step.get("confirmation") == LIVE_CONFIRMATION
                and isinstance(step.get("pid"), int)
            ]
            snapshots = [step for step in read_steps if step.get("action") == "live_sdk_read_only_snapshot"]
            linked = False
            for shutdown in shutdowns:
                shutdown_time = _parse_utc_timestamp(shutdown.get("timestamp"))
                for confirmation in live_confirmations:
                    if shutdown.get("pid") != confirmation.get("pid"):
                        continue
                    confirmation_time = _parse_utc_timestamp(confirmation.get("timestamp"))
                    if confirmation_time is None or shutdown_time is None or confirmation_time > shutdown_time:
                        continue
                    if any(
                        confirmation_time
                        <= snapshot_time
                        <= shutdown_time
                        for snapshot_time in (_parse_utc_timestamp(step.get("timestamp")) for step in snapshots)
                        if snapshot_time is not None
                    ):
                        linked = True
                        break
                if linked:
                    break
            if not linked:
                reasons.append("normal_exit shutdown is not linked to a confirmed read-only SDK session")
        elif scenario == "ctrl_c":
            if not any(step.get("signal") == signal.SIGINT for step in matching("keyboard_interrupt_received")):
                reasons.append("ctrl_c lacks SIGINT evidence")
            reasons.extend(_monitor_evidence_reasons(steps, scenario, None))
        elif scenario == "process_kill":
            reasons.extend(_monitor_evidence_reasons(steps, scenario, PROCESS_KILL_CONFIRMATION))
            events = matching("manual_process_kill_verified")
            monitor_identities = {
                (step.get("pid"), step.get("process_start_ticks"))
                for step in matching("failure_monitor_started")
                if isinstance(step.get("pid"), int) and isinstance(step.get("process_start_ticks"), int)
            }
            if not any(
                step.get("operator_confirmation") == PROCESS_KILL_VERIFIED_CONFIRMATION
                and step.get("process_absent") is True
                and (step.get("checked_monitor_pid"), step.get("checked_process_start_ticks"))
                in monitor_identities
                and step.get("observed_process_start_ticks") != step.get("checked_process_start_ticks")
                for step in events
            ):
                reasons.append("process_kill lacks confirmed absent-process evidence")
        elif scenario == "network_disconnect":
            reasons.extend(_monitor_evidence_reasons(steps, scenario, NETWORK_DISCONNECT_CONFIRMATION))
            events = matching("physical_network_disconnect_verified")
            if not any(
                step.get("operator_confirmation") == NETWORK_DISCONNECT_VERIFIED_CONFIRMATION for step in events
            ):
                reasons.append("network_disconnect lacks exact verified-event confirmation")
            absence = matching("monitor_process_absent_after_disconnect")
            shutdown_outcomes = matching("sdk_shutdown_returned_after_monitor") + matching("sdk_shutdown_failed")
            monitor_identities = {
                (step.get("pid"), step.get("process_start_ticks"))
                for step in matching("failure_monitor_started")
                if isinstance(step.get("pid"), int) and isinstance(step.get("process_start_ticks"), int)
            }
            if not shutdown_outcomes and not any(
                step.get("operator_confirmation") == NETWORK_MONITOR_ABSENT_CONFIRMATION
                and step.get("process_absent") is True
                and (step.get("checked_monitor_pid"), step.get("checked_process_start_ticks"))
                in monitor_identities
                and step.get("observed_process_start_ticks") != step.get("checked_process_start_ticks")
                for step in absence
            ):
                reasons.append("network_disconnect lacks SDK shutdown or confirmed absent-process evidence")
        return reasons

    @staticmethod
    def _update_completion(report: dict[str, Any]) -> None:
        reasons = []
        for scenario in REQUIRED_SCENARIOS:
            scenario_data = report.get("scenarios", {}).get(scenario, {})
            state = scenario_data.get("status", "MISSING")
            if state != "COMPLETE":
                reasons.append(f"{scenario} is {state}")
                continue
            actions = {step.get("action") for step in scenario_data.get("steps", [])}
            for group in SCENARIO_REQUIRED_ACTION_GROUPS[scenario]:
                if not actions.intersection(group):
                    reasons.append(f"{scenario} is missing required runtime evidence")
                    break
            required_phases = SCENARIO_REQUIRED_CAPTURE_PHASES[scenario]
            captures = [
                item
                for item in report.get("ros_captures", [])
                if item.get("scenario") == scenario and item.get("phase") in required_phases
            ]
            valid_phases = {
                item.get("phase")
                for item in captures
                if ros_capture_evidence_valid(item.get("capture", {}))
                and item.get("operator_confirmation") == OBSERVATION_CONFIRMATION
            }
            if valid_phases != required_phases:
                reasons.append(f"{scenario} is missing valid required ROS captures")
            reasons.extend(M7ReportStore._capture_timing_reasons(report, scenario))
            reasons.extend(M7ReportStore._runtime_evidence_reasons(report, scenario))
            observation_steps = [
                step
                for step in scenario_data.get("steps", [])
                if step.get("action") == "onsite_observation"
                and step.get("operator_confirmation") == OBSERVATION_CONFIRMATION
                and str(step.get("observation", "")).strip()
            ]
            if not observation_steps:
                reasons.append(f"{scenario} is missing confirmed onsite observation")
        report["completion_reasons"] = reasons
        report["status"] = "COMPLETE" if not reasons else "INCOMPLETE"
        report["m8_permitted"] = report["status"] == "COMPLETE" and all(
            report.get("scenarios", {}).get(scenario, {}).get("disposition") == "safe"
            for scenario in REQUIRED_SCENARIOS
        )


def _line_number(text: str, needle: str) -> int | None:
    for index, line in enumerate(text.splitlines(), start=1):
        if needle in line:
            return index
    return None


def _line_number_after(text: str, anchor: str, needle: str) -> int | None:
    anchor_index = text.find(anchor)
    if anchor_index < 0:
        return None
    relative_line = _line_number(text[anchor_index:], needle)
    if relative_line is None:
        return None
    return text[:anchor_index].count("\n") + relative_line


def _parse_utc_timestamp(value: Any) -> datetime | None:
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return None
    return parsed.astimezone(timezone.utc)


def _command_result_valid(capture: Mapping[str, Any], command: list[str]) -> bool:
    return any(
        item.get("command") == command
        and item.get("returncode") == 0
        and bool(str(item.get("stdout", "")).strip())
        for item in capture.get("commands", [])
        if isinstance(item, Mapping)
    )


def ros_capture_evidence_valid(capture: Any) -> bool:
    if not isinstance(capture, Mapping) or capture.get("valid") is not True:
        return False
    required_commands = [
        ["ros2", "service", "list", "-t"],
        *[["ros2", "topic", "info", topic, "--verbose"] for topic in ENDPOINT_TOPICS],
        *[["ros2", "topic", "echo", "--once", topic] for topic in ENDPOINT_TOPICS],
    ]
    return all(_command_result_valid(capture, command) for command in required_commands)


def _snapshot_evidence_reasons(snapshot: Any) -> list[str]:
    if not isinstance(snapshot, Mapping):
        return ["snapshot is not a mapping"]
    reasons = []
    if _parse_utc_timestamp(snapshot.get("timestamp")) is None:
        reasons.append("snapshot timestamp is invalid")
    for field in ("control_rights", "robot_alive", "robot_alive_at_sdk_initialization", "robot_alive_live_check"):
        if not isinstance(snapshot.get(field), bool):
            reasons.append(f"snapshot {field} is not boolean")
    if snapshot.get("robot_alive") != snapshot.get("robot_alive_live_check"):
        reasons.append("snapshot live robot fields disagree")
    if not isinstance(snapshot.get("robot_mode"), str) or not snapshot.get("robot_mode"):
        reasons.append("snapshot robot mode is missing")
    names = snapshot.get("names")
    if not isinstance(names, list) or len(names) != 2 or len(set(names)) != 2 or not all(isinstance(name, str) and name for name in names):
        reasons.append("snapshot must name exactly two distinct arms")
    if not isinstance(snapshot.get("frame"), str) or not snapshot.get("frame"):
        reasons.append("snapshot frame is missing")
    desired = snapshot.get("desired_pose")
    current = snapshot.get("current_pose")
    try:
        desired_valid = [_validate_pose_list(pose, f"evidence desired[{index}]") for index, pose in enumerate(desired)]
        current_valid = [_validate_pose_list(pose, f"evidence current[{index}]") for index, pose in enumerate(current)]
        if len(desired_valid) != 2 or len(current_valid) != 2:
            raise RuntimeError("pose count")
    except (RuntimeError, TypeError):
        reasons.append("snapshot desired/current poses are invalid")
        desired_valid = current_valid = None
    errors = snapshot.get("desired_current_position_error_m")
    if (
        not isinstance(errors, list)
        or len(errors) != 2
        or not all(isinstance(value, (int, float)) and math.isfinite(float(value)) and float(value) >= 0 for value in errors)
    ):
        reasons.append("snapshot desired/current errors are invalid")
    elif desired_valid is not None:
        expected = [
            float(np.linalg.norm(np.asarray(desired_valid[index][:3]) - np.asarray(current_valid[index][:3])))
            for index in range(2)
        ]
        if not all(math.isclose(float(errors[index]), expected[index], abs_tol=1e-9) for index in range(2)):
            reasons.append("snapshot desired/current errors disagree with poses")
    if snapshot.get("status_topics") != list(ENDPOINT_TOPICS):
        reasons.append("snapshot status topic list is incomplete")
    return reasons


def _static_hold_evidence_reasons(step: Mapping[str, Any]) -> list[str]:
    if step.get("confirmation") != HOLD_CONFIRMATION:
        return ["static hold confirmation is invalid"]
    result = step.get("result")
    if not isinstance(result, Mapping):
        return ["static hold result is missing"]
    reasons = []
    if result.get("command_count") != 1:
        reasons.append("static hold command count is not one")
    if result.get("exactly_equal_to_final_desired") is not True:
        reasons.append("static hold target equality is unproven")
    if result.get("final_control_rights") is not True or result.get("final_robot_alive") is not True:
        reasons.append("static hold final rights/alive state is invalid")
    if result.get("final_robot_mode") != "safe":
        reasons.append("static hold final robot mode is not safe")
    threshold = result.get("max_desired_current_error_m")
    if (
        not isinstance(threshold, (int, float))
        or not math.isfinite(float(threshold))
        or not 0.0 <= float(threshold) <= MAX_STATIC_HOLD_DESIRED_CURRENT_ERROR_M
    ):
        reasons.append("static hold threshold evidence is invalid")
    sent_names = result.get("sent_names")
    sent_pose = result.get("sent_pose")
    before = result.get("before")
    if _snapshot_evidence_reasons(before):
        reasons.append("static hold pre-send snapshot is invalid")
    else:
        if before.get("control_rights") is not True or before.get("robot_alive") is not True:
            reasons.append("static hold pre-send rights/alive state is invalid")
        if before.get("robot_mode") != "safe":
            reasons.append("static hold pre-send robot mode is not safe")
        if sent_names != before.get("names"):
            reasons.append("static hold sent names differ from snapshot")
        if sent_pose != before.get("desired_pose"):
            reasons.append("static hold sent pose differs from desired pose")
        if max(before.get("desired_current_position_error_m", [math.inf])) > float(threshold):
            reasons.append("static hold desired/current error exceeds recorded threshold")
    try:
        poses = [_validate_pose_list(pose, f"evidence sent[{index}]") for index, pose in enumerate(sent_pose)]
        if len(poses) != 2:
            raise RuntimeError("pose count")
    except (RuntimeError, TypeError):
        reasons.append("static hold sent poses are invalid")
    return reasons


def _monitor_evidence_reasons(
    steps: list[Mapping[str, Any]],
    scenario: str,
    expected_confirmation: str | None,
) -> list[str]:
    reasons = []
    initial = [step.get("snapshot") for step in steps if step.get("action") == "failure_monitor_initial_snapshot"]
    if not any(
        not _snapshot_evidence_reasons(snapshot)
        and snapshot.get("control_rights") is True
        and snapshot.get("robot_alive") is True
        and snapshot.get("robot_mode") == "safe"
        for snapshot in initial
        if isinstance(snapshot, Mapping)
    ):
        reasons.append(f"{scenario} lacks a safe live monitor preflight snapshot")
    started = [step for step in steps if step.get("action") == "failure_monitor_started"]
    if not any(
        isinstance(step.get("pid"), int)
        and step.get("pid") > 0
        and isinstance(step.get("process_start_ticks"), int)
        and step.get("process_start_ticks") > 0
        and step.get("scenario_confirmation") == expected_confirmation
        and isinstance(step.get("instructions"), str)
        and bool(step.get("instructions"))
        for step in started
    ):
        reasons.append(f"{scenario} monitor start evidence is invalid")
    return reasons


def inspect_sdk_source(sdk_root: str | Path) -> dict[str, Any]:
    root = Path(sdk_root).expanduser().resolve()
    interface_path = root / "astribot_sdk/core/astribot_api/astribot_interface.py"
    client_path = root / "astribot_sdk/core/astribot_api/astribot_client.py"
    if not interface_path.is_file() or not client_path.is_file():
        raise FileNotFoundError(f"Astribot SDK sources not found below {root}")
    interface = interface_path.read_text(encoding="utf-8")
    client = client_path.read_text(encoding="utf-8")
    acquire_start = interface.find("    def acquire_control_rights")
    acquire_end = interface.find("    def transfer_control_rights", acquire_start)
    acquire_source = interface[acquire_start:acquire_end]
    fallback_start = acquire_source.find("        except Exception as e:")
    fallback_source = acquire_source[fallback_start:]
    findings = {
        "sdk_root": str(root),
        "inspected_at": utc_now(),
        "source_sha256": {
            str(interface_path.relative_to(root)): hashlib.sha256(interface.encode()).hexdigest(),
            str(client_path.relative_to(root)): hashlib.sha256(client.encode()).hexdigest(),
        },
        "constructor_acquires_control_rights": {
            "value": "self.acquire_control_rights(high_control_rights)" in interface,
            "source": str(interface_path),
            "line": _line_number(interface, "self.acquire_control_rights(high_control_rights)"),
            "risk": "SDK initialization is not read-only and may stop/restart robot control state.",
        },
        "control_rights_service": {
            "name": "/astribot/control_rights",
            "source": str(interface_path),
            "line": _line_number(interface, 'control_rights_name = "/astribot/control_rights"'),
        },
        "control_rights_exception_fallback_claims_rights": {
            "value": fallback_start >= 0 and "self.have_control_rights = True" in fallback_source,
            "source": str(interface_path),
            "line": None
            if fallback_start < 0
            else interface[:acquire_start].count("\n")
            + acquire_source[:fallback_start].count("\n")
            + (_line_number(fallback_source, "self.have_control_rights = True") or 0),
            "risk": "Even when the audit refuses the vendor 'yes' prompt, an SDK exception path may create the control-rights service and mark this client as owning control.",
        },
        "heartbeat_period_sec": {
            "value": 0.1,
            "source": str(interface_path),
            "line": _line_number(interface, "self.heartbeat_timer = self.node.create_timer(0.1"),
        },
        "heartbeat_payload": {
            "value": [16000000],
            "source": str(interface_path),
            "line": _line_number(interface, "self._pub_heartbeat(data=[16000000])"),
        },
        "shutdown_releases_control_rights": {
            "value": "self.transfer_control_rights(req, resp)" in interface,
            "source": str(interface_path),
            "line": _line_number(interface, "self.transfer_control_rights(req, resp)"),
        },
        "shutdown_destroys_heartbeat_timer": {
            "value": "self.node.destroy_timer(self.heartbeat_timer)" in interface,
            "source": str(interface_path),
            "line": _line_number(interface, "self.node.destroy_timer(self.heartbeat_timer)"),
        },
        "atexit_registration_disabled": {
            "value": "# atexit.register(self.shutdown)" in interface,
            "source": str(interface_path),
            "line": _line_number(interface, "# atexit.register(self.shutdown)"),
            "risk": "SIGKILL and abnormal interpreter exit cannot be assumed to execute shutdown().",
        },
        "destructor_is_not_shutdown": {
            "value": "def __del__(self):" in interface,
            "source": str(interface_path),
            "line": _line_number(interface, "def __del__(self):"),
            "risk": "__del__ destroys node resources but does not explicitly transfer control rights or destroy heartbeat timer.",
        },
        "set_cartesian_pose_signature": {
            "source": str(client_path),
            "line": _line_number(client, "def set_cartesian_pose(self, names:"),
            "required_policy": "Only an exact reread of current desired left/right poses is allowed in M7.",
        },
    }
    return findings


def validate_live_environment(store: M7ReportStore, sdk_root: str | Path) -> dict[str, Any]:
    expected = {
        "ROS_DOMAIN_ID": "25",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROBOT_TYPE": "S1",
    }
    mismatches = {
        name: {"expected": expected_value, "actual": os.getenv(name)}
        for name, expected_value in expected.items()
        if os.getenv(name) != expected_value
    }
    if mismatches:
        details = ", ".join(
            f"{name}={values['actual']!r} (expected {values['expected']!r})"
            for name, values in mismatches.items()
        )
        raise RuntimeError(f"live SDK environment check failed: {details}")
    report = store.load()
    recorded_hashes = report.get("static_sdk_findings", {}).get("source_sha256")
    if not recorded_hashes:
        raise RuntimeError("run the M7 inspect command before live SDK initialization")
    current_findings = inspect_sdk_source(sdk_root)
    if current_findings["source_sha256"] != recorded_hashes:
        raise RuntimeError("Astribot SDK source hashes changed since inspect; rerun inspect and review findings")
    store.initialize(current_findings)
    return current_findings


def _validate_pose_list(value: Any, name: str) -> list[float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (7,) or not np.isfinite(array).all():
        raise RuntimeError(f"{name} must be a finite 7-element Cartesian pose")
    quaternion_norm = float(np.linalg.norm(array[3:]))
    if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-3):
        raise RuntimeError(f"{name} quaternion norm {quaternion_norm:.6f} is invalid")
    return array.tolist()


class M7SDKBoundary:
    def __init__(
        self,
        robot_factory: Callable[..., Any] | None = None,
        freq_hz: float = 100.0,
        sdk_root: str | Path | None = None,
    ):
        self.robot_factory = robot_factory
        self.freq_hz = float(freq_hz)
        self.sdk_root = None if sdk_root is None else Path(sdk_root).expanduser().resolve()
        self.robot = None
        self.names: list[str] = []
        self.frame = "chassis"

    def initialize(self) -> None:
        if self.robot is not None:
            return
        factory = self.robot_factory or self._load_factory()
        with refuse_vendor_force_takeover():
            robot = factory(freq=self.freq_hz)
        self.robot = robot
        try:
            names = [str(robot.arm_left_name), str(robot.arm_right_name)]
            frame = str(robot.chassis_frame_name)
            if len(set(names)) != 2 or not all(name.strip() for name in names):
                raise RuntimeError("SDK must expose two distinct non-empty arm names")
            if not frame.strip():
                raise RuntimeError("SDK must expose a non-empty chassis frame name")
            self.names = names
            self.frame = frame
        except Exception:
            try:
                self.shutdown()
            except Exception as shutdown_exc:
                raise RuntimeError(
                    f"SDK metadata validation failed and cleanup also failed: {shutdown_exc}"
                ) from shutdown_exc
            raise

    def snapshot(self) -> dict[str, Any]:
        robot = self._require_robot()
        rights = bool(robot.get_control_rights_status())
        interface = getattr(robot, "astribot_interface", None)
        live_alive_check = getattr(interface, "is_alive", None)
        if not callable(live_alive_check):
            raise RuntimeError("Astribot interface does not expose a live is_alive() check")
        initial_alive = bool(robot.is_alive)
        live_alive = bool(live_alive_check())
        get_mode = getattr(interface, "get_robot_mode", None)
        if not callable(get_mode):
            raise RuntimeError("Astribot interface does not expose get_robot_mode()")
        mode = get_mode()
        desired_raw = robot.get_desired_cartesian_pose(names=self.names, frame=self.frame)
        current_raw = robot.get_current_cartesian_pose(names=self.names, frame=self.frame)
        if not isinstance(desired_raw, (list, tuple)) or not isinstance(current_raw, (list, tuple)):
            raise RuntimeError("SDK Cartesian pose responses must be lists")
        if len(desired_raw) != 2 or len(current_raw) != 2:
            raise RuntimeError("SDK must return exactly left and right arm poses")
        desired = [_validate_pose_list(pose, f"desired[{index}]") for index, pose in enumerate(desired_raw)]
        current = [_validate_pose_list(pose, f"current[{index}]") for index, pose in enumerate(current_raw)]
        errors = [float(np.linalg.norm(np.asarray(desired[index][:3]) - np.asarray(current[index][:3]))) for index in range(2)]
        return {
            "timestamp": utc_now(),
            "control_rights": rights,
            "robot_alive": live_alive,
            "robot_alive_at_sdk_initialization": initial_alive,
            "robot_alive_live_check": live_alive,
            "robot_mode": mode,
            "names": list(self.names),
            "frame": self.frame,
            "desired_pose": desired,
            "current_pose": current,
            "desired_current_position_error_m": errors,
            "status_topics": list(ENDPOINT_TOPICS),
        }

    def send_exact_desired_hold(
        self,
        max_desired_current_error_m: float = MAX_STATIC_HOLD_DESIRED_CURRENT_ERROR_M,
    ) -> dict[str, Any]:
        threshold = float(max_desired_current_error_m)
        if not math.isfinite(threshold) or threshold < 0.0 or threshold > MAX_STATIC_HOLD_DESIRED_CURRENT_ERROR_M:
            raise RuntimeError(
                f"static-hold desired/current threshold must be between 0 and "
                f"{MAX_STATIC_HOLD_DESIRED_CURRENT_ERROR_M:.3f} m"
            )
        before = self.snapshot()
        if not before["control_rights"]:
            raise RuntimeError("control rights are unavailable")
        if not before["robot_alive"]:
            raise RuntimeError("robot is not alive")
        if before["robot_mode"] != "safe":
            raise RuntimeError(f"robot mode must be safe, got {before['robot_mode']!r}")
        if max(before["desired_current_position_error_m"]) > threshold:
            raise RuntimeError("desired/current position error exceeds static-hold threshold")
        robot = self._require_robot()
        reread_raw = robot.get_desired_cartesian_pose(names=self.names, frame=self.frame)
        if not isinstance(reread_raw, (list, tuple)) or len(reread_raw) != 2:
            raise RuntimeError("final desired reread must contain exactly left and right arm poses")
        reread = [_validate_pose_list(pose, f"final_desired[{index}]") for index, pose in enumerate(reread_raw)]
        if reread != before["desired_pose"]:
            raise RuntimeError("desired pose changed between safety snapshot and final reread")
        interface = getattr(robot, "astribot_interface", None)
        final_rights = bool(robot.get_control_rights_status())
        final_alive = bool(interface.is_alive())
        final_mode = interface.get_robot_mode()
        if not final_rights:
            raise RuntimeError("control rights were lost before final static-hold send")
        if not final_alive:
            raise RuntimeError("robot became not alive before final static-hold send")
        if final_mode != "safe":
            raise RuntimeError(f"robot mode changed before final send: {final_mode!r}")
        exact_target = deepcopy(reread)
        robot.set_cartesian_pose(
            list(self.names),
            exact_target,
            control_way="filter",
            use_wbc=False,
            add_default_torso=True,
        )
        return {
            "timestamp": utc_now(),
            "before": before,
            "sent_names": list(self.names),
            "sent_pose": exact_target,
            "exactly_equal_to_final_desired": exact_target == reread,
            "final_control_rights": final_rights,
            "final_robot_alive": final_alive,
            "final_robot_mode": final_mode,
            "max_desired_current_error_m": threshold,
            "command_count": 1,
        }

    def shutdown(self) -> None:
        robot = self.robot
        self.robot = None
        if robot is None:
            return
        interface = getattr(robot, "astribot_interface", None)
        shutdown = getattr(interface, "shutdown", None)
        if not callable(shutdown):
            raise RuntimeError("Astribot interface does not expose shutdown()")
        shutdown()

    def _load_factory(self):
        if self.sdk_root is None:
            raise RuntimeError("M7 live SDK import requires an explicit SDK root")
        prepare_vendor_robotics_import(self.sdk_root)
        module = import_module("astribot_sdk.core.astribot_api.astribot_client")
        return module.Astribot

    def _require_robot(self):
        if self.robot is None:
            raise RuntimeError("M7 SDK boundary is not initialized")
        return self.robot


def capture_ros_read_only(timeout_sec: float = 5.0, runner: Callable[..., Any] = subprocess.run) -> dict[str, Any]:
    commands = [
        ["ros2", "node", "list"],
        ["ros2", "service", "list", "-t"],
        ["ros2", "service", "type", "/astribot/control_rights"],
        ["ros2", "topic", "list", "-t"],
        *[["ros2", "topic", "info", topic, "--verbose"] for topic in ENDPOINT_TOPICS],
        *[["ros2", "topic", "echo", "--once", topic] for topic in ENDPOINT_TOPICS],
    ]
    results = []
    for command in commands:
        try:
            completed = runner(command, capture_output=True, text=True, timeout=timeout_sec, check=False)
            results.append(
                {
                    "command": command,
                    "returncode": int(completed.returncode),
                    "stdout": completed.stdout,
                    "stderr": completed.stderr,
                }
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            results.append({"command": command, "returncode": None, "stdout": "", "stderr": str(exc)})
    topic_list = next(
        (
            result.get("stdout", "")
            for result in results
            if result.get("command") == ["ros2", "topic", "list", "-t"] and result.get("returncode") == 0
        ),
        "",
    )
    service_list = next(
        (
            result.get("stdout", "")
            for result in results
            if result.get("command") == ["ros2", "service", "list", "-t"] and result.get("returncode") == 0
        ),
        "",
    )
    heartbeat_topics = [
        line.split()[0]
        for line in topic_list.splitlines()
        if "heartbeat" in line.lower() and line.strip().startswith("/")
    ]
    for topic in heartbeat_topics:
        command = ["ros2", "topic", "info", topic, "--verbose"]
        try:
            completed = runner(command, capture_output=True, text=True, timeout=timeout_sec, check=False)
            results.append(
                {
                    "command": command,
                    "returncode": int(completed.returncode),
                    "stdout": completed.stdout,
                    "stderr": completed.stderr,
                }
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            results.append({"command": command, "returncode": None, "stdout": "", "stderr": str(exc)})
    required_commands = [
        ["ros2", "service", "list", "-t"],
        *[["ros2", "topic", "info", topic, "--verbose"] for topic in ENDPOINT_TOPICS],
        *[["ros2", "topic", "echo", "--once", topic] for topic in ENDPOINT_TOPICS],
    ]
    failures = []
    for required in required_commands:
        result = next((item for item in results if item.get("command") == required), None)
        if result is None or result.get("returncode") != 0 or not str(result.get("stdout", "")).strip():
            failures.append(" ".join(required))
    return {
        "captured_at": utc_now(),
        "valid": not failures,
        "validation_failures": failures,
        "control_rights_service_present": any(
            line.split()[0] == "/astribot/control_rights"
            for line in service_list.splitlines()
            if line.strip().startswith("/")
        ),
        "heartbeat_topics": heartbeat_topics,
        "commands": results,
    }


@contextmanager
def refuse_vendor_force_takeover():
    original_input = builtins.input

    def refuse(prompt: str = "") -> str:
        if prompt:
            print(prompt, flush=True)
        print("M7 audit policy: refusing vendor force-takeover prompt; continuing without control rights.", flush=True)
        return ""

    builtins.input = refuse
    try:
        yield
    finally:
        builtins.input = original_input

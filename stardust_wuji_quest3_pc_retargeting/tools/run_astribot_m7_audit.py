from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence

import yaml

from stardust_wuji_quest3_pc_retargeting.hardware_audit.m7_audit import (
    HOLD_CONFIRMATION,
    LIVE_CONFIRMATION,
    OBSERVATION_CONFIRMATION,
    PROCESS_KILL_CONFIRMATION,
    NETWORK_DISCONNECT_CONFIRMATION,
    PROCESS_KILL_VERIFIED_CONFIRMATION,
    NETWORK_DISCONNECT_VERIFIED_CONFIRMATION,
    NETWORK_MONITOR_ABSENT_CONFIRMATION,
    REQUIRED_SCENARIOS,
    M7ReportStore,
    M7SDKBoundary,
    MAX_STATIC_HOLD_DESIRED_CURRENT_ERROR_M,
    capture_ros_read_only,
    inspect_sdk_source,
    require_confirmation,
    ros_capture_evidence_valid,
    prepare_vendor_robotics_import,
    utc_now,
    validate_live_environment,
    _process_start_ticks,
)


DEFAULT_SDK_ROOT = "/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master"
DEFAULT_REPORT = "logs/m7_hardware_audit/report.yaml"


def _print_yaml(value) -> None:
    print(yaml.safe_dump(value, sort_keys=False, allow_unicode=True), flush=True)


def _require_live_args(args: argparse.Namespace) -> None:
    if not args.enable_live_sdk:
        raise RuntimeError("live SDK initialization requires --enable-live-sdk")
    require_confirmation(
        "Confirm onsite: physical E-stop is in hand, robot workspace is clear, arms carry no load, and no competing controller is active.",
        LIVE_CONFIRMATION,
    )


def _initialize_live(args: argparse.Namespace, store: M7ReportStore) -> M7SDKBoundary:
    if not args.enable_live_sdk:
        raise RuntimeError("live SDK initialization requires --enable-live-sdk")
    validate_live_environment(store, args.sdk_root)
    _require_live_args(args)
    boundary = M7SDKBoundary(freq_hz=100.0, sdk_root=args.sdk_root)
    store.append_step(
        args.scenario,
        {
            "action": "operator_confirmed_before_sdk_initialization",
            "confirmation": LIVE_CONFIRMATION,
            "pid": os.getpid(),
        },
    )
    try:
        boundary.initialize()
    except Exception as exc:
        store.append_step(args.scenario, {"action": "sdk_initialization_failed", "error": repr(exc)})
        raise RuntimeError(f"Astribot SDK initialization failed: {exc}") from exc
    return boundary


def _shutdown_and_record(boundary: M7SDKBoundary, store: M7ReportStore, scenario: str, action: str) -> None:
    try:
        boundary.shutdown()
    except Exception as exc:
        store.append_step(scenario, {"action": "sdk_shutdown_failed", "error": repr(exc)})
        raise RuntimeError(f"Astribot SDK shutdown failed: {exc}") from exc
    store.append_step(scenario, {"action": action, "success": True, "pid": os.getpid()})


def command_inspect(args: argparse.Namespace) -> int:
    store = M7ReportStore(args.report)
    findings = inspect_sdk_source(args.sdk_root)
    report = store.initialize(findings)
    _print_yaml({"report": str(store.path), "status": report["status"], "static_sdk_findings": findings})
    return 0


def command_preflight(args: argparse.Namespace) -> int:
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise RuntimeError("preflight timeout must be a finite positive number")
    store = M7ReportStore(args.report)
    validate_live_environment(store, args.sdk_root)
    import_command = [
        sys.executable,
        "-c",
        (
            "from stardust_wuji_quest3_pc_retargeting.hardware_audit.m7_audit "
            "import prepare_vendor_robotics_import; "
            f"prepare_vendor_robotics_import({str(Path(args.sdk_root).expanduser().resolve())!r}); "
            "from astribot_sdk.core.astribot_api.astribot_client import Astribot; "
            "print('Astribot SDK import OK')"
        ),
    ]
    try:
        completed = subprocess.run(
            import_command,
            capture_output=True,
            text=True,
            timeout=args.timeout_sec,
            check=False,
        )
        import_result = {
            "returncode": int(completed.returncode),
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }
    except subprocess.TimeoutExpired as exc:
        import_result = {
            "returncode": None,
            "stdout": exc.stdout or "",
            "stderr": f"SDK import timed out after {args.timeout_sec:.1f} s",
        }
    result = {
        "python": sys.executable,
        "h5py_available": importlib.util.find_spec("h5py") is not None,
        "sdk_import_constructed_robot": False,
        "sdk_import": import_result,
        "ready_for_live_confirmation": import_result["returncode"] == 0,
    }
    _print_yaml(result)
    return 0 if result["ready_for_live_confirmation"] else 2


def command_status(args: argparse.Namespace) -> int:
    report = M7ReportStore(args.report).load()
    _print_yaml(
        {
            "report": str(Path(args.report).expanduser().resolve()),
            "status": report["status"],
            "completion_reasons": report["completion_reasons"],
            "scenarios": {name: data["status"] for name, data in report["scenarios"].items()},
            "m8_permitted": report.get("m8_permitted", False),
        }
    )
    return 0 if report["status"] == "COMPLETE" else 2


def command_next_step(args: argparse.Namespace) -> int:
    report = M7ReportStore(args.report).load()
    workflow_order = (
        "sdk_read_only",
        "normal_exit",
        "static_hold",
        "ctrl_c",
        "process_kill",
        "network_disconnect",
    )
    for scenario in workflow_order:
        if report["scenarios"][scenario]["status"] != "COMPLETE":
            command, safety_gate = _next_scenario_action(report, scenario)
            _print_yaml(
                {
                    "report": str(Path(args.report).expanduser().resolve()),
                    "status": report["status"],
                    "next_scenario": scenario,
                    "next_command": (
                        "python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit "
                        f"--report {args.report} {command}"
                    ),
                    "safety_gate": safety_gate,
                    "m8_permitted": False,
                }
            )
            return 2
    _print_yaml({"status": report["status"], "next_scenario": None, "m8_permitted": report["m8_permitted"]})
    return 0 if report["m8_permitted"] else 2


def _scenario_actions(report: dict, scenario: str) -> set[str]:
    return {
        step.get("action")
        for step in report.get("scenarios", {}).get(scenario, {}).get("steps", [])
    }


def _has_valid_capture(report: dict, scenario: str, phase: str) -> bool:
    return any(
        item.get("scenario") == scenario
        and item.get("phase") == phase
        and ros_capture_evidence_valid(item.get("capture", {}))
        for item in report.get("ros_captures", [])
    )


def _next_scenario_action(report: dict, scenario: str) -> tuple[str, str]:
    actions = _scenario_actions(report, scenario)
    observation = f'record-observation --scenario {scenario} --disposition unknown --observation "REPLACE WITH ACTUAL ONSITE OBSERVATION"'
    if scenario == "sdk_read_only":
        if "live_sdk_read_only_snapshot" not in actions:
            return (
                "live-read --enable-live-sdk",
                "Requires physical E-stop in hand, clear unloaded workspace, stationary safe-mode robot, and no competing controller.",
            )
        if not _has_valid_capture(report, scenario, "during"):
            if "sdk_shutdown_returned" in _scenario_actions(report, "normal_exit"):
                return (
                    "live-read --enable-live-sdk",
                    "The prior SDK session already shut down without a valid during capture; repeat the confirmed read-only session and run next-step from terminal 2 while it remains open.",
                )
            return (
                "ros-capture --scenario sdk_read_only --phase during",
                "Run from terminal 2 while the confirmed live-read session remains open; this command is ROS read-only.",
            )
        if "sdk_shutdown_returned" not in _scenario_actions(report, "normal_exit"):
            return (
                "status",
                "Return to the live-read terminal and enter its requested confirmation so normal SDK shutdown can finish; do not start another SDK client.",
            )
        return observation, "Record only the literal onsite behavior observed during SDK initialization and pose reads."
    if scenario == "normal_exit":
        if "sdk_shutdown_returned" not in actions:
            return "live-read --enable-live-sdk", "A confirmed read-only SDK session must end through shutdown before normal-exit evidence exists."
        if not _has_valid_capture(report, scenario, "after"):
            return (
                "ros-capture --scenario normal_exit --phase after",
                "Capture before starting any replacement SDK client so the post-shutdown graph remains attributable.",
            )
        return observation, "Record only the post-shutdown physical and Orin behavior observed onsite."
    if scenario == "static_hold":
        if "exact_desired_static_hold" not in actions:
            return (
                "static-hold --enable-live-sdk --enable-static-hold",
                "Sends one dual-arm batch exactly equal to a final desired reread; hard desired/current error limit is 0.02 m.",
            )
        if not _has_valid_capture(report, scenario, "after"):
            return "ros-capture --scenario static_hold --phase after", "Capture the post-command endpoint and ROS graph state read-only."
        return observation, "Record the literal desired/current and physical behavior after the one static-hold command."
    if scenario == "ctrl_c":
        if not _has_valid_capture(report, scenario, "before"):
            return "ros-capture --scenario ctrl_c --phase before", "Take the read-only baseline before starting the monitor."
        if "keyboard_interrupt_received" not in actions:
            return "monitor --scenario ctrl_c --enable-live-sdk", "Press Ctrl+C only in this monitor terminal after snapshots are visible."
        if not _has_valid_capture(report, scenario, "after"):
            return "ros-capture --scenario ctrl_c --phase after", "Capture the read-only post-Ctrl+C state before another SDK session."
        return observation, "Record the literal Ctrl+C shutdown and physical/Orin behavior."
    if scenario == "process_kill":
        if not _has_valid_capture(report, scenario, "before"):
            return "ros-capture --scenario process_kill --phase before", "Take the read-only baseline before starting the kill monitor."
        if "failure_monitor_started" not in actions:
            return "monitor --scenario process_kill --enable-live-sdk", "Use only the PID printed after the dedicated manual-SIGKILL confirmation."
        if "manual_process_kill_verified" not in actions:
            return "record-failure-event --scenario process_kill", "Run only after kill -9 and physical observation; the original process identity must be absent."
        if not _has_valid_capture(report, scenario, "after"):
            return "ros-capture --scenario process_kill --phase after", "Capture before starting a replacement SDK client."
        return observation, "Record the literal SIGKILL and Orin/physical behavior without inferring shutdown."
    if not _has_valid_capture(report, scenario, "before"):
        return "ros-capture --scenario network_disconnect --phase before", "Take the read-only baseline before starting the network monitor."
    if "failure_monitor_started" not in actions:
        return "monitor --scenario network_disconnect --enable-live-sdk", "Disconnect only the approved physical control-PC link after its dedicated confirmation."
    if "physical_network_disconnect_verified" not in actions:
        return "record-failure-event --scenario network_disconnect", "Run after reconnection and literal onsite observation of the physical disconnect."
    if not _has_valid_capture(report, scenario, "after"):
        return "ros-capture --scenario network_disconnect --phase after", "Capture after reconnection and before another SDK session."
    outcomes = {"sdk_shutdown_returned_after_monitor", "sdk_shutdown_failed", "monitor_process_absent_after_disconnect"}
    if not actions.intersection(outcomes):
        return (
            "record-monitor-absence",
            "Use only if the original monitor self-exited; if it remains open, return there and press Ctrl+C instead.",
        )
    return observation, "Record the literal network-loss and recovery behavior, including whether shutdown ran or the process self-exited."


def command_ros_capture(args: argparse.Namespace) -> int:
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise RuntimeError("ROS capture timeout must be a finite positive number")
    store = M7ReportStore(args.report)
    confirmation = require_confirmation(
        f"Confirm this is the {args.phase} read-only ROS capture for onsite scenario {args.scenario}.",
        OBSERVATION_CONFIRMATION,
    )
    capture = capture_ros_read_only(timeout_sec=args.timeout_sec)
    entry = store.add_ros_capture(args.scenario, args.phase, capture, confirmation)
    _print_yaml(entry)
    if not capture.get("valid", False):
        print(
            "M7 ROS CAPTURE INCOMPLETE: " + "; ".join(capture.get("validation_failures", [])),
            file=sys.stderr,
            flush=True,
        )
        return 2
    return 0


def command_live_read(args: argparse.Namespace) -> int:
    store = M7ReportStore(args.report)
    args.scenario = "sdk_read_only"
    boundary = _initialize_live(args, store)
    shutdown_ok = False
    try:
        snapshot = boundary.snapshot()
        store.append_step("sdk_read_only", {"action": "live_sdk_read_only_snapshot", "snapshot": snapshot})
        _print_yaml(snapshot)
        require_confirmation(
            "Keep this SDK session open while a second terminal completes ros-capture --scenario sdk_read_only --phase during. Continue only after the capture and onsite observation are complete.",
            OBSERVATION_CONFIRMATION,
        )
        store.append_step("sdk_read_only", {"action": "operator_confirmed_during_capture_complete"})
    finally:
        _shutdown_and_record(boundary, store, "normal_exit", "sdk_shutdown_returned")
        shutdown_ok = True
    print("SDK read-only session ended through interface.shutdown().", flush=True)
    print("Record onsite observations separately with record-observation for sdk_read_only and normal_exit.", flush=True)
    return 0 if shutdown_ok else 1


def command_static_hold(args: argparse.Namespace) -> int:
    if not args.enable_static_hold:
        raise RuntimeError("static hold requires --enable-static-hold in addition to --enable-live-sdk")
    store = M7ReportStore(args.report)
    args.scenario = "static_hold"
    boundary = _initialize_live(args, store)
    try:
        preflight = boundary.snapshot()
        store.append_step("static_hold", {"action": "pre_static_hold_snapshot", "snapshot": preflight})
        require_confirmation(
            "Final confirmation: send one dual-arm command exactly equal to the freshly reread desired poses. No other target is permitted.",
            HOLD_CONFIRMATION,
        )
        result = boundary.send_exact_desired_hold(args.max_desired_current_error_m)
        store.append_step(
            "static_hold",
            {"action": "exact_desired_static_hold", "confirmation": HOLD_CONFIRMATION, "result": result},
        )
        _print_yaml(result)
    finally:
        _shutdown_and_record(boundary, store, "normal_exit", "sdk_shutdown_returned_after_static_hold")
    print("Static-hold command sent once and SDK shut down. Record onsite observation separately.", flush=True)
    return 0


def command_monitor(args: argparse.Namespace) -> int:
    if args.scenario not in {"ctrl_c", "process_kill", "network_disconnect"}:
        raise RuntimeError("monitor scenario must be ctrl_c, process_kill, or network_disconnect")
    if not math.isfinite(args.snapshot_interval_sec) or args.snapshot_interval_sec <= 0.0:
        raise RuntimeError("snapshot interval must be a finite positive number")
    store = M7ReportStore(args.report)
    boundary = _initialize_live(args, store)
    last_snapshot_time = 0.0
    interrupted = False
    try:
        # rclpy replaces Python's SIGINT handler during SDK initialization.
        signal.signal(signal.SIGINT, signal.default_int_handler)
        store.append_step(
            args.scenario,
            {"action": "python_sigint_handler_restored", "signal": int(signal.SIGINT)},
        )
        initial_snapshot = boundary.snapshot()
        if not initial_snapshot["control_rights"]:
            raise RuntimeError("failure monitor requires confirmed control rights")
        if not initial_snapshot["robot_alive"]:
            raise RuntimeError("failure monitor requires a live robot")
        if initial_snapshot["robot_mode"] != "safe":
            raise RuntimeError(f"failure monitor requires safe mode, got {initial_snapshot['robot_mode']!r}")
        store.append_step(args.scenario, {"action": "failure_monitor_initial_snapshot", "snapshot": initial_snapshot})
        scenario_confirmation = _confirm_failure_action(args.scenario)
        instructions = _monitor_instructions(args.scenario, os.getpid())
        store.append_step(
            args.scenario,
            {
                "action": "failure_monitor_started",
                "pid": os.getpid(),
                "process_start_ticks": _process_start_ticks(os.getpid()),
                "scenario_confirmation": scenario_confirmation,
                "instructions": instructions,
            },
        )
        print(instructions, flush=True)
        last_snapshot_time = time.monotonic()
        while True:
            now = time.monotonic()
            if now - last_snapshot_time >= args.snapshot_interval_sec:
                snapshot = boundary.snapshot()
                store.append_step(args.scenario, {"action": "live_monitor_snapshot", "snapshot": snapshot})
                print(
                    json.dumps(
                        {
                            "time": snapshot["timestamp"],
                            "control_rights": snapshot["control_rights"],
                            "robot_alive": snapshot["robot_alive"],
                            "mode": snapshot["robot_mode"],
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                last_snapshot_time = now
            time.sleep(0.1)
    except KeyboardInterrupt:
        interrupted = True
        store.append_step(args.scenario, {"action": "keyboard_interrupt_received", "signal": int(signal.SIGINT)})
    finally:
        _shutdown_and_record(boundary, store, args.scenario, "sdk_shutdown_returned_after_monitor")
    if interrupted:
        print("Ctrl+C received; SDK shutdown returned. Record the physical robot/Orin observation next.", flush=True)
        return 0
    return 1


def _confirm_failure_action(scenario: str) -> str | None:
    if scenario == "process_kill":
        return require_confirmation(
            "Confirm the before capture is valid and the operator is ready to perform a manual SIGKILL from a second terminal.",
            PROCESS_KILL_CONFIRMATION,
        )
    if scenario == "network_disconnect":
        return require_confirmation(
            "Confirm the before capture is valid and the operator is ready to physically disconnect only the approved control-PC network link.",
            NETWORK_DISCONNECT_CONFIRMATION,
        )
    return None


def command_record_observation(args: argparse.Namespace) -> int:
    store = M7ReportStore(args.report)
    confirmation = require_confirmation(
        f"Confirm the following observation was made onsite for {args.scenario}, not inferred from source code.",
        OBSERVATION_CONFIRMATION,
    )
    store.complete_scenario(args.scenario, args.observation, confirmation, args.disposition)
    report = store.load()
    _print_yaml(
        {
            "scenario": args.scenario,
            "scenario_status": report["scenarios"][args.scenario]["status"],
            "report_status": report["status"],
            "m8_permitted": report.get("m8_permitted", False),
            "remaining": report["completion_reasons"],
        }
    )
    return 0


def command_record_failure_event(args: argparse.Namespace, process_exists_fn=None) -> int:
    phrase = {
        "process_kill": PROCESS_KILL_VERIFIED_CONFIRMATION,
        "network_disconnect": NETWORK_DISCONNECT_VERIFIED_CONFIRMATION,
    }[args.scenario]
    confirmation = require_confirmation(
        f"Confirm the {args.scenario} action was physically performed onsite and its immediate behavior was observed.",
        phrase,
    )
    entry = M7ReportStore(args.report).record_failure_event(
        args.scenario,
        confirmation,
        process_exists_fn=process_exists_fn,
    )
    _print_yaml(entry)
    return 0


def command_record_monitor_absence(args: argparse.Namespace, process_exists_fn=None) -> int:
    confirmation = require_confirmation(
        "Confirm the network-disconnect monitor exited by itself and is no longer running; do not use this if it was manually stopped.",
        NETWORK_MONITOR_ABSENT_CONFIRMATION,
    )
    entry = M7ReportStore(args.report).record_network_monitor_absence(
        confirmation,
        process_exists_fn=process_exists_fn,
    )
    _print_yaml(entry)
    return 0


def _monitor_instructions(scenario: str, pid: int) -> str:
    if scenario == "ctrl_c":
        return "Observe desired/current poses, then press Ctrl+C in this terminal. Do not move the robot."
    if scenario == "process_kill":
        return (
            f"First run ros-capture --scenario process_kill --phase before. Then, from a second terminal, run: kill -9 {pid}. "
            "After verifying the physical robot and Orin behavior, run the after capture and record-observation."
        )
    return (
        "First run ros-capture --scenario network_disconnect --phase before. Then physically disconnect only the approved control-PC network link. "
        "Do not run software network commands from this tool. After reconnecting, run the after capture and record-observation."
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Astribot S1 M7 control-rights and failure-behavior audit.")
    parser.add_argument("--report", default=DEFAULT_REPORT)
    parser.add_argument("--sdk-root", default=DEFAULT_SDK_ROOT)
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser("inspect", help="Static SDK source audit only; never imports SDK.")
    inspect_parser.set_defaults(function=command_inspect)
    preflight_parser = subparsers.add_parser(
        "preflight",
        help="Import-check SDK and Python dependencies in a subprocess without constructing Astribot.",
    )
    preflight_parser.add_argument("--timeout-sec", type=float, default=15.0)
    preflight_parser.set_defaults(function=command_preflight)
    status_parser = subparsers.add_parser("status", help="Show report completion status.")
    status_parser.set_defaults(function=command_status)
    next_parser = subparsers.add_parser("next-step", help="Show the next gated onsite audit action without initializing SDK.")
    next_parser.set_defaults(function=command_next_step)

    capture_parser = subparsers.add_parser("ros-capture", help="Run read-only ros2 graph/topic captures.")
    capture_parser.add_argument("--scenario", choices=REQUIRED_SCENARIOS, required=True)
    capture_parser.add_argument("--phase", choices=("before", "during", "after"), required=True)
    capture_parser.add_argument("--timeout-sec", type=float, default=5.0)
    capture_parser.set_defaults(function=command_ros_capture)

    live_parser = subparsers.add_parser("live-read", help="Confirmed live SDK initialization and pose reads only.")
    live_parser.add_argument("--enable-live-sdk", action="store_true")
    live_parser.set_defaults(function=command_live_read, scenario="sdk_read_only")

    hold_parser = subparsers.add_parser("static-hold", help="Send exactly one target equal to final desired poses.")
    hold_parser.add_argument("--enable-live-sdk", action="store_true")
    hold_parser.add_argument("--enable-static-hold", action="store_true")
    hold_parser.add_argument(
        "--max-desired-current-error-m",
        type=float,
        default=MAX_STATIC_HOLD_DESIRED_CURRENT_ERROR_M,
        help="May only tighten the hard 0.02 m safety limit; larger or non-finite values fail closed.",
    )
    hold_parser.set_defaults(function=command_static_hold, scenario="static_hold")

    monitor_parser = subparsers.add_parser("monitor", help="Live monitor for Ctrl+C, SIGKILL, or cable-disconnect audit.")
    monitor_parser.add_argument("--enable-live-sdk", action="store_true")
    monitor_parser.add_argument("--scenario", choices=("ctrl_c", "process_kill", "network_disconnect"), required=True)
    monitor_parser.add_argument("--snapshot-interval-sec", type=float, default=1.0)
    monitor_parser.set_defaults(function=command_monitor)

    observation_parser = subparsers.add_parser("record-observation", help="Record verified onsite behavior and complete one scenario.")
    observation_parser.add_argument("--scenario", choices=REQUIRED_SCENARIOS, required=True)
    observation_parser.add_argument("--observation", required=True)
    observation_parser.add_argument("--disposition", choices=("safe", "unsafe", "unknown"), required=True)
    observation_parser.set_defaults(function=command_record_observation)

    event_parser = subparsers.add_parser(
        "record-failure-event",
        help="Record that manual SIGKILL or physical network disconnect actually occurred.",
    )
    event_parser.add_argument("--scenario", choices=("process_kill", "network_disconnect"), required=True)
    event_parser.set_defaults(function=command_record_failure_event)

    absence_parser = subparsers.add_parser(
        "record-monitor-absence",
        help="Verify and record that the network-disconnect monitor process is absent.",
    )
    absence_parser.set_defaults(function=command_record_monitor_absence)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.function(args))
    except RuntimeError as exc:
        print(f"M7 AUDIT STOPPED: {exc}", file=sys.stderr, flush=True)
        return 2
    except KeyboardInterrupt:
        print("M7 AUDIT CANCELLED: no confirmed action was taken.", file=sys.stderr, flush=True)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

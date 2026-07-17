from __future__ import annotations

import argparse
import signal
import socket
import subprocess
import sys
import time
from typing import Sequence


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Start the recording-only ROS2 Hand Bridge and interactive Control-PC "
            "Supervisor with one shared lifecycle."
        )
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--real",
        dest="real",
        action="store_true",
        help="real dual-arm fixed-anchor mode (default)",
    )
    mode.add_argument(
        "--dry-run",
        dest="real",
        action="store_false",
        help="no real arm motion; real hand Retargeter only",
    )
    parser.set_defaults(real=True)
    parser.add_argument("--supervisor-host", default="0.0.0.0")
    parser.add_argument("--supervisor-port", type=int, default=9001)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=9011)
    parser.add_argument("--position-scale", type=float, default=2.0)
    parser.add_argument("--rotation-scale", type=float, default=1.0)
    parser.add_argument("--hand-control-rate-hz", type=float, default=120.0)
    parser.add_argument(
        "extra_supervisor_args",
        nargs=argparse.REMAINDER,
        help="extra Supervisor arguments after --",
    )
    args = parser.parse_args(argv)
    for label, port in (
        ("supervisor", args.supervisor_port),
        ("bridge", args.bridge_port),
    ):
        if not 1 <= int(port) <= 65535:
            parser.error(f"{label} port must be in [1, 65535]")
    if args.hand_control_rate_hz <= 0.0:
        parser.error("hand control rate must be positive")
    if args.position_scale <= 0.0 or args.rotation_scale <= 0.0:
        parser.error("position and rotation scales must be positive")
    if args.extra_supervisor_args[:1] == ["--"]:
        args.extra_supervisor_args = args.extra_supervisor_args[1:]
    return args


def ensure_port_available(host: str, port: int, socket_type: int, label: str) -> None:
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    handle = socket.socket(family, socket_type)
    try:
        handle.bind((host, int(port)))
    except OSError as exc:
        raise RuntimeError(
            f"{label} {host}:{port} is unavailable; stop the old process first: {exc}"
        ) from exc
    finally:
        handle.close()


def build_commands(args: argparse.Namespace) -> tuple[list[str], list[str]]:
    bridge = [
        sys.executable,
        "-m",
        "stardust_wuji_quest3_pc_retargeting.tools.run_wujihand_ros2_bridge",
        "--listen-host",
        str(args.bridge_host),
        "--listen-port",
        str(args.bridge_port),
    ]
    supervisor = [
        sys.executable,
        "-m",
        "stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor",
        "--host",
        str(args.supervisor_host),
        "--port",
        str(args.supervisor_port),
        "--arm",
        "both",
        "--mapping-mode",
        "relative",
        "--enable-hand-dryrun",
        "--hand-retarget-real",
        "--hand-control-rate-hz",
        str(args.hand_control_rate_hz),
        "--hand-bridge-udp-host",
        str(args.bridge_host),
        "--hand-bridge-udp-port",
        str(args.bridge_port),
    ]
    if args.real:
        supervisor.extend(
            [
                "--run-m8-fixed-anchor-real",
                "--m8-position-scale",
                str(args.position_scale),
                "--m8-rotation-scale",
                str(args.rotation_scale),
            ]
        )
    else:
        supervisor.extend(["--dry-run", "--interactive"])
    supervisor.extend(args.extra_supervisor_args)
    return bridge, supervisor


def stop_process(process: subprocess.Popen | None, timeout: float = 3.0) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    ensure_port_available(
        args.supervisor_host,
        args.supervisor_port,
        socket.SOCK_STREAM,
        "Supervisor TCP endpoint",
    )
    ensure_port_available(
        args.bridge_host,
        args.bridge_port,
        socket.SOCK_DGRAM,
        "Hand Bridge UDP endpoint",
    )
    bridge_command, supervisor_command = build_commands(args)
    bridge = None
    supervisor = None
    try:
        print(
            f"[full-teleop] Starting Hand Bridge on "
            f"udp://{args.bridge_host}:{args.bridge_port}; "
            "real WujiHand commands are DISABLED.",
            flush=True,
        )
        bridge = subprocess.Popen(bridge_command)
        time.sleep(1.0)
        if bridge.poll() is not None:
            raise RuntimeError(
                f"Hand Bridge exited during startup with code {bridge.returncode}"
            )

        if args.real:
            print(
                "[full-teleop] Starting REAL dual-arm fixed-anchor Supervisor; "
                "physical emergency stop must be ready.",
                flush=True,
            )
        else:
            print(
                "[full-teleop] Starting DRY-RUN Supervisor; real arms will not move.",
                flush=True,
            )
        print(
            f"[full-teleop] Orin must forward WebXR to "
            f"ws://<CONTROL_PC_IP>:{args.supervisor_port}",
            flush=True,
        )
        supervisor = subprocess.Popen(supervisor_command)
        return int(supervisor.wait())
    except KeyboardInterrupt:
        return 130
    finally:
        stop_process(supervisor)
        stop_process(bridge)
        print("[full-teleop] Supervisor and Hand Bridge stopped.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

import argparse
import asyncio
import json
import time
from copy import deepcopy
from math import isfinite
from pathlib import Path
from typing import Sequence

import websockets

from stardust_wuji_quest3_pc_retargeting.protocol.json_codec import encode_message
from stardust_wuji_quest3_pc_retargeting.hardware_audit.m8_waiver import validate_m8_waiver
from stardust_wuji_quest3_pc_retargeting.runtime.config import load_yaml_config
from stardust_wuji_quest3_pc_retargeting.runtime.control_commands import parse_control_command
from stardust_wuji_quest3_pc_retargeting.runtime.supervisor import ControlPCSupervisor


M8_RISK_BUNDLE_TOKEN = "M8_ACCEPT_ALL_AUTHORIZED_RISKS"


def load_arm_config(service_config_path: str | Path) -> tuple[dict, dict]:
    service = load_yaml_config(service_config_path)
    workspace_config = service.get("arms", {}).get("workspace_config", "configs/arm/s1_quest3_default.yaml")
    arm_config = load_yaml_config(workspace_config)
    return service, arm_config


def build_supervisor(args: argparse.Namespace) -> ControlPCSupervisor:
    bundled_confirmation = args.accept_m8_risk_bundle == M8_RISK_BUNDLE_TOKEN
    if args.accept_m8_risk_bundle is not None and not bundled_confirmation:
        raise RuntimeError("M8 bundled risk confirmation token did not match")
    if args.enable_real_hand:
        raise RuntimeError("M8 does not authorize real hand hardware")
    service, arm_config = load_arm_config(args.config)
    if args.enable_real_arm:
        arm_config = deepcopy(arm_config)
        position_scale = float(args.m8_position_scale)
        if not isfinite(position_scale) or position_scale <= 0.0:
            raise RuntimeError("M8 position scale must be finite and positive")
        arm_config.setdefault("mapping", {})["position_scale_xyz"] = [position_scale] * 3
        safety = arm_config.setdefault("safety", {})
        requested_speed = float(args.m8_max_linear_speed_mps)
        if not 0.0 < requested_speed <= 1.00:
            raise RuntimeError("M8 max linear speed must be in (0, 1.0] m/s")
        if requested_speed > 0.20 and not (args.confirm_m8_high_speed or bundled_confirmation):
            raise RuntimeError("M8 speed above 0.20 m/s requires --confirm-m8-high-speed")
        safety["max_linear_speed_mps"] = requested_speed
        safety["max_input_position_jump_m"] = 0.03
        safety["mode_switch_max_position_jump_m"] = 0.01
        hand_reacquire_timeout = float(args.m8_hand_reacquire_timeout_sec)
        if not isfinite(hand_reacquire_timeout) or hand_reacquire_timeout < 0.0:
            raise RuntimeError("M8 hand reacquire timeout must be finite and non-negative")
        safety["hand_reacquire_timeout_sec"] = hand_reacquire_timeout
        safety["hand_reacquire_stable_frames"] = 3
        if args.enable_m8_absolute_orientation_reacquire:
            if not args.enable_m8_orientation:
                raise RuntimeError("absolute orientation reacquire requires --enable-m8-orientation")
            if hand_reacquire_timeout <= 0.0:
                raise RuntimeError("absolute orientation reacquire requires a positive hand reacquire timeout")
            recovery_speed = float(args.m8_orientation_reacquire_speed_rad_s)
            recovery_max_error = float(args.m8_orientation_reacquire_max_error_rad)
            if not isfinite(recovery_speed) or recovery_speed <= 0.0:
                raise RuntimeError("orientation reacquire speed must be finite and positive")
            if not isfinite(recovery_max_error) or not 0.15 <= recovery_max_error <= 3.141592653589793:
                raise RuntimeError("orientation reacquire max error must be finite and in [0.15, pi]")
            safety["absolute_orientation_reacquire"] = True
            safety["orientation_reacquire_speed_rad_s"] = recovery_speed
            safety["orientation_reacquire_direct_error_rad"] = 0.15
            safety["orientation_reacquire_complete_error_rad"] = 0.087
            safety["orientation_reacquire_max_error_rad"] = recovery_max_error
            safety["orientation_reacquire_complete_frames"] = 5
        position_alpha = float(args.m8_position_alpha)
        if not 0.0 < position_alpha <= 1.0:
            raise RuntimeError("M8 position alpha must be in (0, 1]")
        arm_config.setdefault("filter", {})["position_alpha"] = position_alpha
        if args.enable_m8_orientation:
            if not (args.confirm_m8_orientation or bundled_confirmation):
                raise RuntimeError("M8 orientation requires --confirm-m8-orientation")
            rotation_scale = float(args.m8_rotation_scale)
            angular_speed = float(args.m8_max_angular_speed_rad_s)
            if not isfinite(rotation_scale) or not 0.0 < rotation_scale <= 1.0:
                raise RuntimeError("M8 rotation scale must be finite and in (0, 1]")
            if not isfinite(angular_speed) or angular_speed <= 0.0:
                raise RuntimeError("M8 max angular speed must be finite and positive")
            if args.enable_m8_absolute_orientation_reacquire and rotation_scale != 1.0:
                raise RuntimeError("absolute orientation reacquire requires --m8-rotation-scale 1.0")
            if (rotation_scale > 0.30 or angular_speed > 0.30) and not (
                args.confirm_m8_high_rate_orientation or bundled_confirmation
            ):
                raise RuntimeError(
                    "M8 orientation above the initial 0.30 limits requires --confirm-m8-high-rate-orientation"
                )
            arm_config["mapping"]["enable_orientation"] = True
            arm_config["mapping"]["rotation_scale"] = rotation_scale
            safety["max_angular_speed_rad_s"] = angular_speed
            safety["max_input_rotation_jump_rad"] = 0.35
    selected_mode = args.mapping_mode or arm_config.get("mapping", {}).get("mode", "relative")
    if args.enable_real_arm:
        if not (args.confirm_m8_real_arm or bundled_confirmation):
            raise RuntimeError("M8 real arm requires --confirm-m8-real-arm")
        if not args.m8_waiver:
            raise RuntimeError("M8 real arm requires --m8-waiver")
        waiver = validate_m8_waiver(
            args.m8_waiver,
            arm_config,
            args.arm,
            selected_mode,
            require_control_takeover=args.allow_control_takeover,
        )
        if bundled_confirmation:
            if waiver.get("attestation", {}).get("bundled_confirmation_token") != M8_RISK_BUNDLE_TOKEN:
                raise RuntimeError("M8 waiver does not authorize bundled confirmation")
        else:
            expected_arm_phrase = (
                "M8 BOTH ARMS RELATIVE PHYSICAL ESTOP"
                if args.arm == "both"
                else "M8 LEFT RELATIVE PHYSICAL ESTOP"
            )
            phrase = input(f"Type exactly: {expected_arm_phrase}\n> ").strip()
            if phrase != expected_arm_phrase:
                raise RuntimeError("M8 physical safety confirmation did not match")
            if args.allow_control_takeover:
                takeover_phrase = input("Type exactly: M8 FORCE TAKEOVER WEB CONTROL RIGHTS\n> ").strip()
                if takeover_phrase != "M8 FORCE TAKEOVER WEB CONTROL RIGHTS":
                    raise RuntimeError("M8 control-rights takeover confirmation did not match")
            if float(args.m8_max_linear_speed_mps) > 0.20:
                expected_speed_phrase = (
                    "M8 HIGH SPEED 1.0 MPS PHYSICAL ESTOP"
                    if float(args.m8_max_linear_speed_mps) > 0.50
                    else "M8 HIGH SPEED 0.5 MPS PHYSICAL ESTOP"
                )
                speed_phrase = input(f"Type exactly: {expected_speed_phrase}\n> ").strip()
                if speed_phrase != expected_speed_phrase:
                    raise RuntimeError("M8 high-speed confirmation did not match")
            if args.enable_m8_orientation:
                orientation_phrase = input("Type exactly: M8 ENABLE ORIENTATION PHYSICAL ESTOP\n> ").strip()
                if orientation_phrase != "M8 ENABLE ORIENTATION PHYSICAL ESTOP":
                    raise RuntimeError("M8 orientation confirmation did not match")
                if float(args.m8_rotation_scale) > 0.30 or float(args.m8_max_angular_speed_rad_s) > 0.30:
                    high_rate_phrase = input("Type exactly: M8 HIGH RATE ORIENTATION PHYSICAL ESTOP\n> ").strip()
                    if high_rate_phrase != "M8 HIGH RATE ORIENTATION PHYSICAL ESTOP":
                        raise RuntimeError("M8 high-rate orientation confirmation did not match")
    return ControlPCSupervisor(
        arm_config,
        arm=args.arm,
        mapping_mode=selected_mode,
        enable_real_arm=args.enable_real_arm,
        absolute_calibration_report=args.absolute_calibration_report,
        sdk_root=service.get("arms", {}).get("sdk_root", "/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master"),
        high_control_rights=args.allow_control_takeover,
        allow_orientation_control=args.enable_m8_orientation,
    )


async def serve_control_pc(supervisor: ControlPCSupervisor, host: str, port: int):
    async def handler(socket):
        last_state_sent = 0.0
        try:
            async for raw in socket:
                payload = json.loads(raw)
                if payload.get("type") == "tracking_frame":
                    supervisor.ingest_payload(payload)
                    seq = payload.get("seq", 0)
                    now = time.monotonic()
                    if now - last_state_sent < 0.2:
                        continue
                elif payload.get("type") == "control_command":
                    command = supervisor.submit_command(payload.get("command", ""), payload.get("argument"))
                    result = await asyncio.to_thread(supervisor.wait_command, command, 2.0)
                    seq = payload.get("seq", 0)
                    await socket.send(
                        encode_message(
                            {
                                "schema": "quest3_web_teleop.v1",
                                "type": "command_result",
                                "seq": seq,
                                "accepted": bool(result and result.accepted),
                                "message": "command timeout" if result is None else result.message,
                            }
                        )
                    )
                    continue
                else:
                    seq = payload.get("seq", 0)
                last_state_sent = time.monotonic()
                await socket.send(
                    encode_message(
                        {
                            "schema": "quest3_web_teleop.v1",
                            "type": "control_state",
                            "seq": seq,
                            **supervisor.status_dict(),
                        }
                    )
                )
        except websockets.exceptions.ConnectionClosed:
            return
        except Exception as exc:
            try:
                await socket.send(
                    encode_message(
                        {
                            "schema": "quest3_web_teleop.v1",
                            "type": "control_error",
                            "message": str(exc),
                        }
                    )
                )
            except websockets.exceptions.ConnectionClosed:
                return

    return await websockets.serve(handler, host, int(port))


async def command_console(supervisor: ControlPCSupervisor) -> None:
    while True:
        try:
            line = await asyncio.to_thread(input, "teleop> ")
        except (EOFError, KeyboardInterrupt):
            return
        if line.strip().lower() in {"quit", "exit"}:
            return
        try:
            name, argument = parse_control_command(line)
            result = await asyncio.to_thread(supervisor.execute_command, name, argument, 2.0)
            print(("OK: " if result.accepted else "REJECTED: ") + result.message, flush=True)
        except Exception as exc:
            print(f"ERROR: {exc}", flush=True)


async def run_server(
    host: str,
    port: int,
    supervisor: ControlPCSupervisor,
    initial_commands: Sequence[str] = (),
    interactive: bool = False,
) -> None:
    supervisor.start()
    server = None
    try:
        if supervisor.adapter.enable_real:
            await asyncio.to_thread(supervisor.wait_until_adapter_ready, 20.0)
        server = await serve_control_pc(supervisor, host, port)
        for text in initial_commands:
            name, argument = parse_control_command(text)
            result = await asyncio.to_thread(supervisor.execute_command, name, argument, 2.0)
            print(("OK: " if result.accepted else "REJECTED: ") + result.message, flush=True)
        if interactive:
            await command_console(supervisor)
        else:
            await server.wait_closed()
    finally:
        if server is not None:
            server.close()
            await server.wait_closed()
        supervisor.close()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Control PC Quest3 arm supervisor (dry-run by default).")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--config", default="configs/services/control_pc_default.yaml")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument("--mapping-mode", choices=("relative", "absolute"), default=None)
    parser.add_argument("--absolute-calibration-report", default=None)
    parser.add_argument("--command", action="append", default=[])
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--dry-run", action="store_true", default=True)
    parser.add_argument("--enable-real-hand", action="store_true")
    parser.add_argument("--enable-real-arm", action="store_true")
    parser.add_argument("--m8-waiver", default=None)
    parser.add_argument("--confirm-m8-real-arm", action="store_true")
    parser.add_argument("--allow-control-takeover", action="store_true")
    parser.add_argument("--m8-max-linear-speed-mps", type=float, default=0.20)
    parser.add_argument("--confirm-m8-high-speed", action="store_true")
    parser.add_argument(
        "--m8-position-scale",
        type=float,
        default=1.0,
        help="uniform Quest-to-robot translation scale; the active waiver sets the maximum",
    )
    parser.add_argument(
        "--m8-hand-reacquire-timeout-sec",
        type=float,
        default=0.0,
        help="hold and automatically re-anchor hand tracking if it returns within this many seconds; 0 disables",
    )
    parser.add_argument("--enable-m8-absolute-orientation-reacquire", action="store_true")
    parser.add_argument("--m8-orientation-reacquire-speed-rad-s", type=float, default=0.5)
    parser.add_argument("--m8-orientation-reacquire-max-error-rad", type=float, default=1.57)
    parser.add_argument("--m8-position-alpha", type=float, default=0.70)
    parser.add_argument("--enable-m8-orientation", action="store_true")
    parser.add_argument("--confirm-m8-orientation", action="store_true")
    parser.add_argument("--confirm-m8-high-rate-orientation", action="store_true")
    parser.add_argument("--m8-rotation-scale", type=float, default=0.30)
    parser.add_argument("--m8-max-angular-speed-rad-s", type=float, default=0.30)
    parser.add_argument(
        "--accept-m8-risk-bundle",
        metavar="TOKEN",
        default=None,
        help=(
            "replace the M8 interactive confirmations with the exact token "
            f"{M8_RISK_BUNDLE_TOKEN}; the waiver must authorize the same token"
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        supervisor = build_supervisor(args)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    asyncio.run(run_server(args.host, args.port, supervisor, args.command, args.interactive))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

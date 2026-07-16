from __future__ import annotations

import argparse
import asyncio
from threading import Event, Thread
from typing import Sequence

from stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor import (
    build_supervisor,
    load_arm_config,
    parse_args as parse_supervisor_args,
    serve_control_pc,
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    return parse_supervisor_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        supervisor = build_supervisor(args)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    supervisor.start()
    stop_server = Event()

    async def web_server() -> None:
        server = await serve_control_pc(supervisor, args.host, args.port)
        try:
            await asyncio.to_thread(stop_server.wait)
        finally:
            server.close()
            await server.wait_closed()

    server_thread = Thread(target=lambda: asyncio.run(web_server()), name="ControlPCWebSocket", daemon=True)
    server_thread.start()
    try:
        from stardust_wuji_quest3_pc_retargeting.ui.control_panel import launch_control_panel

        service_config, _ = load_arm_config(args.config)
        launch_control_panel(
            supervisor,
            pause_on_close=bool(service_config.get("control_panel", {}).get("pause_on_close", True)),
        )
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        stop_server.set()
        server_thread.join(timeout=2.0)
        supervisor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

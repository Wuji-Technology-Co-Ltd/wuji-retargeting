from __future__ import annotations

import argparse
import asyncio
import threading
from pathlib import Path
from typing import Sequence

from stardust_wuji_quest3_pc_retargeting.web_gateway.static_server import make_static_server
from stardust_wuji_quest3_pc_retargeting.web_gateway.websocket_relay import serve_relay


async def run_gateway(
    static_root: str,
    web_host: str,
    web_port: int,
    ws_host: str,
    ws_port: int,
    control_pc_url: str,
    cert_file: str | None = None,
    key_file: str | None = None,
) -> None:
    server = make_static_server(web_host, web_port, static_root, cert_file=cert_file, key_file=key_file)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    relay_server = await serve_relay(ws_host, ws_port, control_pc_url)
    try:
        await asyncio.Future()
    finally:
        relay_server.close()
        await relay_server.wait_closed()
        server.shutdown()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Quest3 static web server and WebSocket relay.")
    parser.add_argument("--static-root", default=str(Path(__file__).resolve().parents[2] / "quest3_web"))
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8443)
    parser.add_argument("--ws-host", default="0.0.0.0")
    parser.add_argument("--ws-port", type=int, default=9002)
    parser.add_argument("--control-pc-url", default="ws://127.0.0.1:9001")
    parser.add_argument("--cert-file", default=None)
    parser.add_argument("--key-file", default=None)
    parser.add_argument("--config", default="configs/services/orin_web_gateway_default.yaml")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    asyncio.run(
        run_gateway(
            args.static_root,
            args.web_host,
            args.web_port,
            args.ws_host,
            args.ws_port,
            args.control_pc_url,
            cert_file=args.cert_file,
            key_file=args.key_file,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

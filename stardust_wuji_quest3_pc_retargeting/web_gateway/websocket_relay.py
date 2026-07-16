from __future__ import annotations

import asyncio

import websockets

from .relay_diagnostics import RelayCounters, record_control_message, record_quest_message


async def _pipe_quest_to_control(source, sink, counters: RelayCounters) -> None:
    async for message in source:
        if record_quest_message(message, counters, lambda line: print(line, flush=True)):
            await sink.send(message)


async def _pipe_control_to_quest(source, sink, counters: RelayCounters) -> None:
    async for message in source:
        await sink.send(message)
        record_control_message(counters, lambda line: print(line, flush=True))


async def relay_websockets(quest_socket, control_pc_url: str, counters: RelayCounters | None = None) -> None:
    counters = counters or RelayCounters()
    print(f"[relay] quest connected; connecting control PC {control_pc_url}", flush=True)
    async with websockets.connect(control_pc_url) as control_socket:
        print("[relay] control PC connected", flush=True)

        await asyncio.gather(
            _pipe_quest_to_control(quest_socket, control_socket, counters),
            _pipe_control_to_quest(control_socket, quest_socket, counters),
        )


async def serve_relay(listen_host: str, listen_port: int, control_pc_url: str):
    async def handler(socket, *_path):
        try:
            await relay_websockets(socket, control_pc_url)
        except Exception as exc:
            print(f"[relay] connection closed/error: {exc}", flush=True)

    print(f"[relay] listening ws://{listen_host}:{int(listen_port)} -> {control_pc_url}", flush=True)
    return await websockets.serve(handler, listen_host, int(listen_port))

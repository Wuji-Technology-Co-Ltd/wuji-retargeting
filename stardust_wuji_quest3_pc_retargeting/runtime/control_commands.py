from __future__ import annotations

from dataclasses import dataclass, field
from queue import Empty, Queue
from threading import Event, Lock
from time import monotonic_ns
from typing import Any
from uuid import uuid4


COMMAND_NAMES = {
    "invalidate-calibration",
    "calibration-status",
    "cancel-calibration",
    "pause",
    "stop",
    "estop",
    "reset",
    "engage",
    "clutch-resume",
    "recover-init",
    "status",
}

COMMAND_ALIASES = {
    "e": "engage",
    "p": "pause",
    "r": "recover-init",
    "s": "status",
}


def parse_control_command(text: str) -> tuple[str, str | None]:
    parts = str(text).strip().lower().split(maxsplit=1)
    if not parts:
        raise ValueError("control command cannot be empty")
    name = parts[0].replace("_", "-")
    name = COMMAND_ALIASES.get(name, name)
    argument = parts[1] if len(parts) == 2 else None
    if name in {"mode", "mapping-mode"}:
        if argument not in {"relative", "absolute"}:
            raise ValueError("mode command requires relative or absolute")
        return "mode", argument
    if name not in COMMAND_NAMES:
        raise ValueError(f"unknown control command: {name}")
    return name, argument


@dataclass(frozen=True)
class ControlCommand:
    name: str
    argument: str | None = None
    command_id: str = field(default_factory=lambda: uuid4().hex)
    submitted_ns: int = field(default_factory=monotonic_ns)


@dataclass(frozen=True)
class CommandResult:
    command_id: str
    accepted: bool
    message: str
    completed_ns: int


class ControlCommandQueue:
    def __init__(self) -> None:
        self._queue: Queue[ControlCommand] = Queue()
        self._results: dict[str, CommandResult] = {}
        self._events: dict[str, Event] = {}
        self._lock = Lock()

    def submit(self, name: str, argument: str | None = None) -> ControlCommand:
        normalized = str(name).strip().lower().replace("_", "-")
        if not normalized:
            raise ValueError("command name cannot be empty")
        normalized = COMMAND_ALIASES.get(normalized, normalized)
        command = ControlCommand(normalized, None if argument is None else str(argument).strip())
        with self._lock:
            self._events[command.command_id] = Event()
        self._queue.put(command)
        return command

    def get_nowait(self) -> ControlCommand | None:
        try:
            return self._queue.get_nowait()
        except Empty:
            return None

    def complete(self, command: ControlCommand, accepted: bool, message: str) -> CommandResult:
        result = CommandResult(command.command_id, bool(accepted), str(message), monotonic_ns())
        with self._lock:
            self._results[command.command_id] = result
            event = self._events.get(command.command_id)
            if event is not None:
                event.set()
        return result

    def wait(self, command: ControlCommand | str, timeout: float | None = None) -> CommandResult | None:
        command_id = command.command_id if isinstance(command, ControlCommand) else str(command)
        with self._lock:
            result = self._results.get(command_id)
            event = self._events.get(command_id)
        if result is not None:
            return result
        if event is None or not event.wait(timeout):
            return None
        with self._lock:
            return self._results.get(command_id)

    def result(self, command: ControlCommand | str) -> CommandResult | None:
        command_id = command.command_id if isinstance(command, ControlCommand) else str(command)
        with self._lock:
            return self._results.get(command_id)

    @property
    def pending_count(self) -> int:
        return self._queue.qsize()

    def snapshot_results(self) -> dict[str, Any]:
        with self._lock:
            return {command_id: result for command_id, result in self._results.items()}

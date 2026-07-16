from __future__ import annotations

from enum import Enum


class TeleopState(str, Enum):
    IDLE = "IDLE"
    ARMED = "ARMED"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    ESTOP = "ESTOP"
    FAULT = "FAULT"


class TeleopStateMachine:
    def __init__(self, start_in: TeleopState | str = TeleopState.IDLE):
        self.state = TeleopState(start_in)
        self.fault_reason = ""

    def arm(self) -> None:
        if self.state in {TeleopState.IDLE, TeleopState.PAUSED}:
            self.state = TeleopState.ARMED
            self.fault_reason = ""
        else:
            self.fault(f"arm requires IDLE or PAUSED, got {self.state.value}")

    def start(self) -> None:
        if self.state is TeleopState.ARMED:
            self.state = TeleopState.RUNNING
            self.fault_reason = ""
        else:
            self.fault(f"start requires ARMED, got {self.state.value}")

    def pause(self) -> None:
        if self.state is TeleopState.RUNNING:
            self.state = TeleopState.PAUSED
        elif self.state not in {TeleopState.PAUSED, TeleopState.IDLE}:
            self.fault(f"pause requires RUNNING, got {self.state.value}")

    def resume(self) -> None:
        if self.state is TeleopState.PAUSED:
            self.state = TeleopState.RUNNING
            self.fault_reason = ""
        else:
            self.fault(f"resume requires PAUSED, got {self.state.value}")

    def stop(self) -> None:
        if self.state is not TeleopState.ESTOP:
            self.state = TeleopState.IDLE
            self.fault_reason = ""

    def estop(self) -> None:
        self.state = TeleopState.ESTOP
        self.fault_reason = "emergency stop"

    def fault(self, reason: str) -> None:
        self.state = TeleopState.FAULT
        self.fault_reason = reason

    def reset(self) -> None:
        self.state = TeleopState.IDLE
        self.fault_reason = ""

    def handle_command(self, command: str) -> None:
        normalized = command.strip().lower()
        if normalized in {"c", "calibrate", "arm"}:
            self.arm()
        elif normalized in {"space", "start"}:
            self.start()
        elif normalized in {"p", "pause", "hold", "h"}:
            self.pause()
        elif normalized in {"r", "resume"}:
            self.resume()
        elif normalized in {"q", "stop", "end"}:
            self.stop()
        elif normalized in {"esc", "estop"}:
            self.estop()
        elif normalized in {"reset", "clear"}:
            self.reset()
        else:
            self.fault(f"unknown command: {command}")

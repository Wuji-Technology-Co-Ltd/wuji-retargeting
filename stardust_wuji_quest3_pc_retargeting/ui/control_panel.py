from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from stardust_wuji_quest3_pc_retargeting.runtime.supervisor import ControlPCSupervisor


BUTTON_COMMANDS = {
    "Engage (E)": ("engage", None),
    "Cancel Calibration": ("cancel-calibration", None),
    "Pause (P)": ("pause", None),
    "Recover Init (R)": ("recover-init", None),
    "Stop": ("stop", None),
    "E-Stop": ("estop", None),
}


@dataclass
class _PanelValue:
    label: str
    getter: Callable[[object], str]


class ControlPanel:
    def __init__(self, supervisor: "ControlPCSupervisor", root=None, pause_on_close: bool = True) -> None:
        self.supervisor = supervisor
        self.pause_on_close = bool(pause_on_close)
        self._tk, self._ttk = self._load_tkinter()
        try:
            self.root = root or self._tk.Tk()
        except self._tk.TclError as exc:
            raise RuntimeError("Tkinter cannot open a display; use the headless CLI instead") from exc
        self.root.title("Quest3 → Astribot S1 Arm Teleop (DRY-RUN)")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._closed = False
        self._status_variables = {}
        self._calibration_text = self._tk.StringVar(value="UNCALIBRATED")
        self._message_text = self._tk.StringVar(value="IDLE — no robot commands")
        self._build()
        self.refresh()

    @staticmethod
    def _load_tkinter():
        try:
            import tkinter as tk
            from tkinter import ttk
        except ImportError as exc:
            raise RuntimeError("Tkinter is unavailable; run the CLI without --panel") from exc
        return tk, ttk

    def _build(self) -> None:
        container = self._ttk.Frame(self.root, padding=12)
        container.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        warning = self._tk.Label(
            container,
            text="DRY-RUN ONLY — Software E-Stop does not replace the physical E-Stop",
            bg="#8b0000",
            fg="white",
            padx=8,
            pady=6,
        )
        warning.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        fields = [
            _PanelValue("Teleop state", lambda s: s.teleop_state),
            _PanelValue("Mapping mode", lambda s: s.mapping_mode),
            _PanelValue("WebXR", lambda s: "active" if s.webxr_active else "inactive"),
            _PanelValue("XR session", lambda s: s.xr_session_id or "—"),
            _PanelValue("Reference space", lambda s: f"{s.reference_space or '—'} rev={s.reference_space_revision}"),
            _PanelValue("HMD tracking", lambda s: "valid" if s.hmd_tracking else "invalid"),
            _PanelValue("Hand tracking", lambda s: ", ".join(f"{k}={'valid' if v else 'invalid'}" for k, v in s.hand_tracking.items())),
            _PanelValue("Robot", lambda s: "dry-run connected" if s.robot_connected else "disconnected"),
            _PanelValue("Control rights", lambda s: "dry-run" if s.control_rights else "unavailable"),
            _PanelValue("Loop", lambda s: f"{s.loop_state}, sent={s.sent_cycles}, misses={s.missed_deadlines}"),
        ]
        for row, field in enumerate(fields, start=1):
            self._ttk.Label(container, text=field.label).grid(row=row, column=0, sticky="w", padx=(0, 12))
            variable = self._tk.StringVar(value="—")
            self._status_variables[field.label] = (variable, field.getter)
            self._ttk.Label(container, textvariable=variable).grid(row=row, column=1, sticky="w")

        calibration_row = len(fields) + 1
        self._ttk.Label(container, text="Calibration").grid(row=calibration_row, column=0, sticky="nw", padx=(0, 12))
        self._ttk.Label(container, textvariable=self._calibration_text, wraplength=620).grid(
            row=calibration_row, column=1, sticky="w"
        )

        button_row = calibration_row + 1
        button_frame = self._ttk.Frame(container)
        button_frame.grid(row=button_row, column=0, columnspan=2, sticky="ew", pady=(12, 8))
        for index, label in enumerate(BUTTON_COMMANDS):
            self._ttk.Button(button_frame, text=label, command=lambda name=label: self.invoke_button(name)).grid(
                row=index // 4, column=index % 4, padx=3, pady=3, sticky="ew"
            )

        self._ttk.Label(container, textvariable=self._message_text, wraplength=760).grid(
            row=button_row + 1, column=0, columnspan=2, sticky="w"
        )

    def invoke_button(self, label: str) -> None:
        command = BUTTON_COMMANDS[label]
        if len(command) == 4:
            mode_name, mode_argument, action_name, action_argument = command
            self.supervisor.submit_command(mode_name, mode_argument)
            self.supervisor.submit_command(action_name, action_argument)
        else:
            name, argument = command
            self.supervisor.submit_command(name, argument)
        self._message_text.set(f"Queued: {label}")

    def refresh(self) -> None:
        if self._closed:
            return
        status = self.supervisor.status_snapshot()
        for variable, getter in self._status_variables.values():
            variable.set(getter(status))
        progress = status.calibration_progress
        calibration = (
            f"{status.calibration_state}; countdown={progress.get('countdown_remaining_sec', 0):.1f}s; "
            f"sampling={float(progress.get('sampling_progress', 0)) * 100:.0f}%; "
            f"samples={progress.get('sample_count', 0)}/{progress.get('minimum_valid_samples', 0)}"
        )
        if status.calibration_failure_reason:
            calibration += f"; FAILED: {status.calibration_failure_reason}"
        if status.calibration_quality:
            quality = ", ".join(f"{key}={value}" for key, value in status.calibration_quality.items())
            calibration += f"; quality: {quality}"
        self._calibration_text.set(calibration)
        self._message_text.set(status.last_error or status.last_command_message or "Ready")
        self.root.after(100, self.refresh)

    def close(self) -> None:
        if self.pause_on_close:
            self.supervisor.submit_command("pause")
        self._closed = True
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def launch_control_panel(supervisor: "ControlPCSupervisor", pause_on_close: bool = True) -> None:
    ControlPanel(supervisor, pause_on_close=pause_on_close).run()

import sys
from pathlib import Path

from stardust_wuji_quest3_pc_retargeting.runtime.control_commands import ControlCommandQueue, parse_control_command
from stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor import parse_args
from stardust_wuji_quest3_pc_retargeting.ui.control_panel import BUTTON_COMMANDS, ControlPanel


class FakeSupervisor:
    def __init__(self):
        self.commands = []

    def submit_command(self, name, argument=None):
        self.commands.append((name, argument))


class FakeText:
    def set(self, value):
        self.value = value


class FakeRoot:
    def destroy(self):
        self.destroyed = True


def test_cli_supports_arm_mapping_and_all_required_commands():
    args = parse_args(["--arm", "right", "--mapping-mode", "absolute", "--command", "absolute-calibrate"])
    assert args.arm == "right"
    assert args.mapping_mode == "absolute"
    for command in [
        "absolute-calibrate",
        "invalidate-calibration",
        "calibration-status",
        "start",
        "pause",
        "stop",
        "estop",
        "recenter",
        "engage",
        "mode relative",
        "mode absolute",
    ]:
        name, argument = parse_control_command(command)
        assert name
        if command.startswith("mode"):
            assert argument in {"relative", "absolute"}


def test_panel_buttons_only_submit_shared_supervisor_commands():
    supervisor = FakeSupervisor()
    panel = ControlPanel.__new__(ControlPanel)
    panel.supervisor = supervisor
    panel._message_text = FakeText()

    for label in BUTTON_COMMANDS:
        panel.invoke_button(label)

    assert ("recenter", None) in supervisor.commands
    assert ("absolute-calibrate", None) in supervisor.commands
    assert ("start", None) in supervisor.commands
    assert ("estop", None) in supervisor.commands


def test_panel_close_policy_submits_pause_and_never_calls_control_objects():
    supervisor = FakeSupervisor()
    panel = ControlPanel.__new__(ControlPanel)
    panel.supervisor = supervisor
    panel.pause_on_close = True
    panel._closed = False
    panel.root = FakeRoot()

    panel.close()

    assert supervisor.commands == [("pause", None)]
    assert panel.root.destroyed is True


def test_control_panel_import_does_not_import_tkinter_eagerly():
    source = (Path(__file__).parents[1] / "stardust_wuji_quest3_pc_retargeting/ui/control_panel.py").read_text()
    prefix = source.split("def _load_tkinter", 1)[0]
    assert "import tkinter as tk" not in prefix


def test_command_queue_is_thread_safe_contract():
    queue = ControlCommandQueue()
    command = queue.submit("start")
    assert queue.pending_count == 1
    assert queue.get_nowait() == command
    result = queue.complete(command, True, "ok")
    assert queue.wait(command, 0.01) == result

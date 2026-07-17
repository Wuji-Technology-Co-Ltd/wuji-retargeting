import sys

from stardust_wuji_quest3_pc_retargeting.tools.run_full_control_pc_stack import (
    build_commands,
    parse_args,
)


def test_full_stack_defaults_to_real_arms_and_recording_only_hand_bridge():
    args = parse_args([])
    bridge, supervisor = build_commands(args)

    assert bridge[:3] == [sys.executable, "-m", bridge[2]]
    assert "--publish-driver-commands" not in bridge
    assert "--run-m8-fixed-anchor-real" in supervisor
    assert "--hand-retarget-real" in supervisor
    assert "--enable-hand-dryrun" in supervisor
    assert "--enable-real-hand" not in supervisor


def test_full_stack_dry_run_supports_temporary_ports_and_extra_args():
    args = parse_args(
        [
            "--dry-run",
            "--supervisor-port",
            "19001",
            "--bridge-port",
            "19011",
            "--",
            "--command",
            "status",
        ]
    )
    bridge, supervisor = build_commands(args)

    assert "--run-m8-fixed-anchor-real" not in supervisor
    assert "--dry-run" in supervisor
    assert "--interactive" in supervisor
    assert supervisor[-2:] == ["--command", "status"]
    assert bridge[-1] == "19011"

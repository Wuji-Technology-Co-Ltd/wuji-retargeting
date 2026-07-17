from stardust_wuji_quest3_pc_retargeting.tools import run_control_pc_supervisor as cli


def test_control_pc_cli_builds_formal_dual_hand_dryrun_pipeline():
    args = cli.parse_args(["--enable-hand-dryrun"])

    supervisor = cli.build_supervisor(args)
    try:
        status = supervisor.status_snapshot()
        assert status.hand_pipeline_enabled is True
        assert status.hand_retarget_real is False
        assert status.hand_command_sink == "dry-run"
        assert supervisor.adapter.enable_real is False
    finally:
        supervisor.close()


def test_udp_bridge_option_enables_hand_pipeline_without_real_hand_flag():
    args = cli.parse_args(["--hand-bridge-udp-port", "19011"])

    supervisor = cli.build_supervisor(args)
    try:
        status = supervisor.status_snapshot()
        assert status.hand_pipeline_enabled is True
        assert status.hand_command_sink == "udp://127.0.0.1:19011"
        assert args.enable_real_hand is False
    finally:
        supervisor.close()

from __future__ import annotations

import signal
from copy import deepcopy
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml

from stardust_wuji_quest3_pc_retargeting.hardware_audit.m7_audit import (
    ENDPOINT_TOPICS,
    HOLD_CONFIRMATION,
    LIVE_CONFIRMATION,
    NETWORK_DISCONNECT_CONFIRMATION,
    NETWORK_MONITOR_ABSENT_CONFIRMATION,
    NETWORK_DISCONNECT_VERIFIED_CONFIRMATION,
    OBSERVATION_CONFIRMATION,
    PROCESS_KILL_CONFIRMATION,
    PROCESS_KILL_VERIFIED_CONFIRMATION,
    REQUIRED_SCENARIOS,
    M7ReportStore,
    M7SDKBoundary,
    capture_ros_read_only,
    inspect_sdk_source,
    _process_start_ticks,
    utc_now,
    validate_live_environment,
)
from stardust_wuji_quest3_pc_retargeting.tools import run_astribot_m7_audit as cli


LEFT = [0.45, 0.35, 1.05, 0, 0, 0, 1]
RIGHT = [0.45, -0.35, 1.05, 0, 0, 0, 1]


def valid_snapshot(*, rights=True, alive=True, mode="safe"):
    return {
        "timestamp": utc_now(),
        "control_rights": rights,
        "robot_alive": alive,
        "robot_alive_at_sdk_initialization": alive,
        "robot_alive_live_check": alive,
        "robot_mode": mode,
        "names": ["astribot_arm_left", "astribot_arm_right"],
        "frame": "chassis",
        "desired_pose": deepcopy([LEFT, RIGHT]),
        "current_pose": deepcopy([LEFT, RIGHT]),
        "desired_current_position_error_m": [0.0, 0.0],
        "status_topics": list(ENDPOINT_TOPICS),
    }


def valid_ros_capture():
    commands = [["ros2", "service", "list", "-t"]]
    commands.extend([["ros2", "topic", "info", topic, "--verbose"] for topic in ENDPOINT_TOPICS])
    commands.extend([["ros2", "topic", "echo", "--once", topic] for topic in ENDPOINT_TOPICS])
    return {
        "valid": True,
        "validation_failures": [],
        "commands": [
            {"command": command, "returncode": 0, "stdout": "captured", "stderr": ""}
            for command in commands
        ],
    }


def append_live_confirmation(store, scenario):
    store.append_step(
        scenario,
        {
            "action": "operator_confirmed_before_sdk_initialization",
            "confirmation": LIVE_CONFIRMATION,
            "pid": 1234,
        },
    )


def append_runtime_evidence(store, scenario):
    if scenario == "sdk_read_only":
        append_live_confirmation(store, scenario)
        store.append_step(scenario, {"action": "live_sdk_read_only_snapshot", "snapshot": valid_snapshot()})
    elif scenario == "static_hold":
        append_live_confirmation(store, scenario)
        snapshot = valid_snapshot()
        store.append_step(scenario, {"action": "pre_static_hold_snapshot", "snapshot": snapshot})
        store.append_step(
            scenario,
            {
                "action": "exact_desired_static_hold",
                "confirmation": HOLD_CONFIRMATION,
                "result": {
                    "before": snapshot,
                    "sent_names": snapshot["names"],
                    "sent_pose": snapshot["desired_pose"],
                    "exactly_equal_to_final_desired": True,
                    "final_control_rights": True,
                    "final_robot_alive": True,
                    "final_robot_mode": "safe",
                    "max_desired_current_error_m": 0.02,
                    "command_count": 1,
                },
            },
        )
    elif scenario == "normal_exit":
        read_steps = store.load()["scenarios"]["sdk_read_only"]["steps"]
        if not any(step.get("action") == "live_sdk_read_only_snapshot" for step in read_steps):
            append_runtime_evidence(store, "sdk_read_only")
        store.append_step(scenario, {"action": "sdk_shutdown_returned", "success": True, "pid": 1234})
    else:
        append_live_confirmation(store, scenario)
        store.append_step(
            scenario,
            {"action": "failure_monitor_initial_snapshot", "snapshot": valid_snapshot()},
        )
        confirmation = {
            "ctrl_c": None,
            "process_kill": PROCESS_KILL_CONFIRMATION,
            "network_disconnect": NETWORK_DISCONNECT_CONFIRMATION,
        }[scenario]
        store.append_step(
            scenario,
            {
                "action": "failure_monitor_started",
                "pid": 1234,
                "process_start_ticks": 5678,
                "scenario_confirmation": confirmation,
                "instructions": f"verified instructions for {scenario}",
            },
        )
        if scenario == "ctrl_c":
            store.append_step(scenario, {"action": "keyboard_interrupt_received", "signal": int(signal.SIGINT)})
            store.append_step(scenario, {"action": "sdk_shutdown_returned_after_monitor", "success": True})
        elif scenario == "process_kill":
            store.record_failure_event(
                scenario,
                PROCESS_KILL_VERIFIED_CONFIRMATION,
                process_exists_fn=lambda pid: False,
                process_start_ticks_fn=lambda pid: None,
            )
        else:
            store.record_failure_event(scenario, NETWORK_DISCONNECT_VERIFIED_CONFIRMATION)
            store.append_step(scenario, {"action": "sdk_shutdown_returned_after_monitor", "success": True})


def prepare_scenario(store, scenario, *, include_captures=True):
    phases = {
        "sdk_read_only": ["during"],
        "static_hold": ["after"],
        "normal_exit": ["after"],
        "ctrl_c": ["before", "after"],
        "process_kill": ["before", "after"],
        "network_disconnect": ["before", "after"],
    }[scenario]
    if include_captures and "before" in phases:
        store.add_ros_capture(scenario, "before", valid_ros_capture(), OBSERVATION_CONFIRMATION)
    append_runtime_evidence(store, scenario)
    if include_captures:
        for phase in phases:
            if phase != "before":
                store.add_ros_capture(scenario, phase, valid_ros_capture(), OBSERVATION_CONFIRMATION)


def mock_robot(*, rights=True, alive=True, mode="safe", desired=None, current=None):
    robot = Mock()
    robot.arm_left_name = "astribot_arm_left"
    robot.arm_right_name = "astribot_arm_right"
    robot.chassis_frame_name = "chassis"
    robot.get_control_rights_status.return_value = rights
    robot.is_alive = alive
    robot.astribot_interface.is_alive.return_value = alive
    robot.astribot_interface.get_robot_mode.return_value = mode
    robot.astribot_interface.shutdown = Mock()
    robot.get_desired_cartesian_pose.return_value = deepcopy(desired or [LEFT, RIGHT])
    robot.get_current_cartesian_pose.return_value = deepcopy(current or [LEFT, RIGHT])
    return robot


def test_static_inspection_records_control_heartbeat_and_shutdown_facts():
    findings = inspect_sdk_source("/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master")

    assert findings["constructor_acquires_control_rights"]["value"] is True
    assert findings["control_rights_service"]["name"] == "/astribot/control_rights"
    assert findings["control_rights_exception_fallback_claims_rights"]["value"] is True
    assert findings["heartbeat_period_sec"]["value"] == 0.1
    assert findings["shutdown_releases_control_rights"]["value"] is True
    assert findings["atexit_registration_disabled"]["value"] is True


def test_sdk_boundary_read_snapshot_uses_official_pose_calls_and_shutdown():
    robot = mock_robot()
    factory = Mock(return_value=robot)
    boundary = M7SDKBoundary(robot_factory=factory)
    boundary.initialize()

    snapshot = boundary.snapshot()
    boundary.shutdown()

    factory.assert_called_once_with(freq=100.0)
    robot.get_desired_cartesian_pose.assert_called_once_with(
        names=["astribot_arm_left", "astribot_arm_right"], frame="chassis"
    )
    robot.get_current_cartesian_pose.assert_called_once_with(
        names=["astribot_arm_left", "astribot_arm_right"], frame="chassis"
    )
    assert snapshot["control_rights"] is True
    assert snapshot["robot_alive_live_check"] is True
    assert snapshot["robot_mode"] == "safe"
    robot.astribot_interface.shutdown.assert_called_once_with()


def test_sdk_boundary_refuses_vendor_force_takeover_prompt(monkeypatch):
    calls = []

    def factory(**kwargs):
        calls.append(input("vendor prompt"))
        return mock_robot(rights=False)

    boundary = M7SDKBoundary(robot_factory=factory)
    boundary.initialize()

    assert calls == [""]
    assert boundary.snapshot()["control_rights"] is False


@pytest.mark.parametrize(
    "left_name, right_name, frame, message",
    [
        ("same", "same", "chassis", "distinct"),
        ("", "right", "chassis", "non-empty arm"),
        ("left", "right", "", "chassis frame"),
    ],
)
def test_sdk_boundary_cleans_up_if_metadata_validation_fails(left_name, right_name, frame, message):
    robot = mock_robot()
    robot.arm_left_name = left_name
    robot.arm_right_name = right_name
    robot.chassis_frame_name = frame
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))

    with pytest.raises(RuntimeError, match=message):
        boundary.initialize()

    assert boundary.robot is None
    robot.astribot_interface.shutdown.assert_called_once_with()


def test_static_hold_sends_one_batch_exactly_equal_to_final_desired():
    robot = mock_robot()
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))
    boundary.initialize()

    result = boundary.send_exact_desired_hold()

    assert result["exactly_equal_to_final_desired"] is True
    assert result["final_control_rights"] is True
    assert result["command_count"] == 1
    robot.set_cartesian_pose.assert_called_once_with(
        ["astribot_arm_left", "astribot_arm_right"],
        [LEFT, RIGHT],
        control_way="filter",
        use_wbc=False,
        add_default_torso=True,
    )
    assert robot.get_desired_cartesian_pose.call_count == 2


@pytest.mark.parametrize(
    "robot, message",
    [
        (mock_robot(rights=False), "control rights"),
        (mock_robot(alive=False), "not alive"),
        (mock_robot(mode="professional"), "mode must be safe"),
        (
            mock_robot(current=[[0.40, 0.35, 1.05, 0, 0, 0, 1], RIGHT]),
            "desired/current position error",
        ),
    ],
)
def test_static_hold_fails_closed_before_send(robot, message):
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))
    boundary.initialize()

    with pytest.raises(RuntimeError, match=message):
        boundary.send_exact_desired_hold()

    robot.set_cartesian_pose.assert_not_called()


def test_static_hold_rejects_when_live_alive_check_disagrees_with_initial_state():
    robot = mock_robot(alive=True)
    robot.astribot_interface.is_alive.return_value = False
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))
    boundary.initialize()

    with pytest.raises(RuntimeError, match="not alive"):
        boundary.send_exact_desired_hold()

    robot.set_cartesian_pose.assert_not_called()


def test_static_hold_rejects_desired_pose_change_between_reads():
    robot = mock_robot()
    changed = deepcopy([LEFT, RIGHT])
    changed[0][0] += 0.001
    robot.get_desired_cartesian_pose.side_effect = [deepcopy([LEFT, RIGHT]), changed]
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))
    boundary.initialize()

    with pytest.raises(RuntimeError, match="desired pose changed"):
        boundary.send_exact_desired_hold()

    robot.set_cartesian_pose.assert_not_called()


def test_static_hold_rejects_control_rights_lost_after_final_desired_read():
    robot = mock_robot()
    robot.get_control_rights_status.side_effect = [True, False]
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))
    boundary.initialize()

    with pytest.raises(RuntimeError, match="lost before final"):
        boundary.send_exact_desired_hold()

    robot.set_cartesian_pose.assert_not_called()


@pytest.mark.parametrize("threshold", [0.020001, float("inf"), float("nan"), -0.001])
def test_static_hold_threshold_cannot_weaken_hard_two_centimeter_limit(threshold):
    robot = mock_robot()
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))
    boundary.initialize()

    with pytest.raises(RuntimeError, match="threshold must be between"):
        boundary.send_exact_desired_hold(threshold)

    robot.get_desired_cartesian_pose.assert_not_called()
    robot.set_cartesian_pose.assert_not_called()


def test_static_hold_final_reread_requires_exactly_two_arm_poses():
    robot = mock_robot()
    robot.get_desired_cartesian_pose.side_effect = [deepcopy([LEFT, RIGHT]), deepcopy([LEFT])]
    boundary = M7SDKBoundary(robot_factory=Mock(return_value=robot))
    boundary.initialize()

    with pytest.raises(RuntimeError, match="exactly left and right"):
        boundary.send_exact_desired_hold()

    robot.set_cartesian_pose.assert_not_called()


def test_report_never_completes_until_all_onsite_scenarios_are_verified(tmp_path):
    store = M7ReportStore(tmp_path / "report.yaml")
    store.initialize({"source": "mock"})
    for scenario in REQUIRED_SCENARIOS[:-1]:
        prepare_scenario(store, scenario)
        store.complete_scenario(scenario, f"verified {scenario}", OBSERVATION_CONFIRMATION, "safe")

    incomplete = store.load()
    assert incomplete["status"] == "INCOMPLETE"
    assert incomplete["scenarios"]["network_disconnect"]["status"] == "PENDING"

    prepare_scenario(store, "network_disconnect")
    store.complete_scenario("network_disconnect", "verified disconnect behavior", OBSERVATION_CONFIRMATION, "safe")

    complete = store.load()
    assert complete["status"] == "COMPLETE"
    assert complete["completion_reasons"] == []
    assert complete["m8_permitted"] is True


def test_kill_and_disconnect_observation_requires_before_and_after_capture(tmp_path):
    store = M7ReportStore(tmp_path / "report.yaml")
    store.initialize()
    store.add_ros_capture("process_kill", "before", valid_ros_capture(), OBSERVATION_CONFIRMATION)
    append_runtime_evidence(store, "process_kill")

    with pytest.raises(RuntimeError, match="valid required read-only ROS captures"):
        store.complete_scenario("process_kill", "observed", OBSERVATION_CONFIRMATION, "safe")


@pytest.mark.parametrize("scenario", REQUIRED_SCENARIOS)
def test_scenario_cannot_complete_without_required_runtime_evidence(tmp_path, scenario):
    store = M7ReportStore(tmp_path / f"{scenario}.yaml")
    store.initialize()

    with pytest.raises(RuntimeError, match="missing required audit evidence"):
        store.complete_scenario(scenario, "claimed observation", OBSERVATION_CONFIRMATION, "safe")


def test_scenario_cannot_complete_without_required_ros_capture(tmp_path):
    store = M7ReportStore(tmp_path / "capture.yaml")
    store.initialize()
    append_runtime_evidence(store, "sdk_read_only")

    with pytest.raises(RuntimeError, match="required read-only ROS captures"):
        store.complete_scenario("sdk_read_only", "observed", OBSERVATION_CONFIRMATION, "safe")


def test_scenario_cannot_complete_with_failed_ros_capture(tmp_path):
    store = M7ReportStore(tmp_path / "failed-capture.yaml")
    store.initialize()
    append_runtime_evidence(store, "sdk_read_only")
    store.add_ros_capture(
        "sdk_read_only",
        "during",
        {"valid": False, "validation_failures": ["endpoint topic missing"]},
        OBSERVATION_CONFIRMATION,
    )

    with pytest.raises(RuntimeError, match="missing valid required read-only ROS captures"):
        store.complete_scenario("sdk_read_only", "observed", OBSERVATION_CONFIRMATION, "safe")


def test_failed_ros_capture_can_be_retried_without_deleting_audit_history(tmp_path):
    store = M7ReportStore(tmp_path / "retried-capture.yaml")
    store.initialize()
    append_runtime_evidence(store, "sdk_read_only")
    store.add_ros_capture(
        "sdk_read_only",
        "during",
        {"valid": False, "validation_failures": ["first attempt timed out"]},
        OBSERVATION_CONFIRMATION,
    )
    store.add_ros_capture("sdk_read_only", "during", valid_ros_capture(), OBSERVATION_CONFIRMATION)

    store.complete_scenario("sdk_read_only", "verified after retry", OBSERVATION_CONFIRMATION, "safe")

    report = store.load()
    assert report["scenarios"]["sdk_read_only"]["status"] == "COMPLETE"
    assert len(report["ros_captures"]) == 2


def test_report_store_rejects_unconfirmed_capture_and_observation(tmp_path):
    store = M7ReportStore(tmp_path / "unconfirmed.yaml")
    store.initialize()

    with pytest.raises(RuntimeError, match="capture confirmation"):
        store.add_ros_capture("sdk_read_only", "during", {"valid": True}, "not confirmed")

    store.append_step("sdk_read_only", {"action": "live_sdk_read_only_snapshot"})
    store.add_ros_capture("sdk_read_only", "during", {"valid": True}, OBSERVATION_CONFIRMATION)
    with pytest.raises(RuntimeError, match="observation confirmation"):
        store.complete_scenario("sdk_read_only", "claimed observation", "not confirmed", "safe")


def test_report_rejects_capture_valid_flag_without_endpoint_command_evidence(tmp_path):
    store = M7ReportStore(tmp_path / "empty-capture.yaml")
    store.initialize()
    append_runtime_evidence(store, "sdk_read_only")
    store.add_ros_capture(
        "sdk_read_only",
        "during",
        {"valid": True, "commands": []},
        OBSERVATION_CONFIRMATION,
    )

    with pytest.raises(RuntimeError, match="missing valid required read-only ROS captures"):
        store.complete_scenario("sdk_read_only", "claimed onsite", OBSERVATION_CONFIRMATION, "safe")


def test_report_rejects_static_hold_action_without_exact_single_send_evidence(tmp_path):
    store = M7ReportStore(tmp_path / "weak-hold.yaml")
    store.initialize()
    append_live_confirmation(store, "static_hold")
    store.append_step("static_hold", {"action": "pre_static_hold_snapshot", "snapshot": valid_snapshot()})
    store.append_step(
        "static_hold",
        {
            "action": "exact_desired_static_hold",
            "confirmation": HOLD_CONFIRMATION,
            "result": {"command_count": 2, "exactly_equal_to_final_desired": True},
        },
    )
    store.add_ros_capture("static_hold", "after", valid_ros_capture(), OBSERVATION_CONFIRMATION)

    with pytest.raises(RuntimeError, match="exact-desired single-command evidence"):
        store.complete_scenario("static_hold", "claimed onsite", OBSERVATION_CONFIRMATION, "safe")


def test_complete_audit_with_unknown_disposition_still_blocks_m8(tmp_path):
    store = M7ReportStore(tmp_path / "unknown.yaml")
    store.initialize()
    for scenario in REQUIRED_SCENARIOS:
        prepare_scenario(store, scenario)
        disposition = "unknown" if scenario == "network_disconnect" else "safe"
        store.complete_scenario(scenario, "onsite result", OBSERVATION_CONFIRMATION, disposition)

    report = store.load()
    assert report["status"] == "COMPLETE"
    assert report["m8_permitted"] is False


def test_completed_scenario_observation_and_disposition_are_immutable(tmp_path):
    store = M7ReportStore(tmp_path / "immutable.yaml")
    store.initialize()
    prepare_scenario(store, "sdk_read_only")
    store.complete_scenario("sdk_read_only", "first onsite result", OBSERVATION_CONFIRMATION, "unsafe")

    with pytest.raises(RuntimeError, match="already COMPLETE"):
        store.complete_scenario("sdk_read_only", "replacement result", OBSERVATION_CONFIRMATION, "safe")

    report = store.load()
    assert report["scenarios"]["sdk_read_only"]["observation"] == "first onsite result"
    assert report["scenarios"]["sdk_read_only"]["disposition"] == "unsafe"


def test_report_load_rejects_manually_claimed_completion_without_evidence(tmp_path):
    path = tmp_path / "tampered.yaml"
    report = M7ReportStore(path).load()
    report["status"] = "COMPLETE"
    report["m8_permitted"] = True
    for scenario in REQUIRED_SCENARIOS:
        report["scenarios"][scenario]["status"] = "COMPLETE"
        report["scenarios"][scenario]["disposition"] = "safe"
    path.write_text(yaml.safe_dump(report), encoding="utf-8")

    audited = M7ReportStore(path).load()

    assert audited["status"] == "INCOMPLETE"
    assert audited["m8_permitted"] is False
    assert audited["completion_reasons"]


def test_report_load_downgrades_completion_when_capture_is_removed(tmp_path):
    store = M7ReportStore(tmp_path / "report.yaml")
    store.initialize()
    prepare_scenario(store, "sdk_read_only")
    store.complete_scenario("sdk_read_only", "observed onsite", OBSERVATION_CONFIRMATION, "safe")
    report = yaml.safe_load(store.path.read_text(encoding="utf-8"))
    report["ros_captures"] = []
    store.path.write_text(yaml.safe_dump(report), encoding="utf-8")

    audited = store.load()

    assert audited["status"] == "INCOMPLETE"
    assert "sdk_read_only is missing valid required ROS captures" in audited["completion_reasons"]


def test_report_rejects_before_capture_recorded_after_failure_monitor_started(tmp_path):
    store = M7ReportStore(tmp_path / "late-before.yaml")
    store.initialize()
    append_runtime_evidence(store, "process_kill")
    store.add_ros_capture("process_kill", "before", valid_ros_capture(), OBSERVATION_CONFIRMATION)
    store.add_ros_capture("process_kill", "after", valid_ros_capture(), OBSERVATION_CONFIRMATION)

    with pytest.raises(RuntimeError, match="before capture timestamp is not before"):
        store.complete_scenario("process_kill", "claimed onsite", OBSERVATION_CONFIRMATION, "safe")


def test_failure_event_requires_monitor_and_exact_scenario_confirmation(tmp_path):
    store = M7ReportStore(tmp_path / "failure-event.yaml")
    store.initialize()

    with pytest.raises(RuntimeError, match="monitor evidence is missing"):
        store.record_failure_event(
            "process_kill",
            PROCESS_KILL_VERIFIED_CONFIRMATION,
            process_exists_fn=lambda pid: False,
        )

    store.append_step("process_kill", {"action": "failure_monitor_started", "pid": 1234})
    with pytest.raises(RuntimeError, match="confirmation phrase"):
        store.record_failure_event("process_kill", "not confirmed")

    entry = store.record_failure_event(
        "process_kill",
        PROCESS_KILL_VERIFIED_CONFIRMATION,
        process_exists_fn=lambda pid: False,
        process_start_ticks_fn=lambda pid: None,
    )
    assert entry["action"] == "manual_process_kill_verified"


def test_process_kill_event_rejects_monitor_pid_that_still_exists(tmp_path):
    store = M7ReportStore(tmp_path / "running-kill.yaml")
    store.initialize()
    store.append_step(
        "process_kill",
        {
            "action": "failure_monitor_started",
            "pid": 1234,
            "process_start_ticks": 5678,
            "scenario_confirmation": PROCESS_KILL_CONFIRMATION,
            "instructions": "kill test",
        },
    )

    with pytest.raises(RuntimeError, match="still running"):
        store.record_failure_event(
            "process_kill",
            PROCESS_KILL_VERIFIED_CONFIRMATION,
            process_exists_fn=lambda pid: True,
            process_start_ticks_fn=lambda pid: 5678,
        )


def test_process_start_ticks_reads_current_process_identity():
    import os

    ticks = _process_start_ticks(os.getpid())

    assert isinstance(ticks, int)
    assert ticks > 0


def test_network_monitor_absence_requires_verified_disconnect_and_absent_pid(tmp_path):
    store = M7ReportStore(tmp_path / "network-absence.yaml")
    store.initialize()
    store.append_step(
        "network_disconnect",
        {
            "action": "failure_monitor_started",
            "pid": 1234,
            "process_start_ticks": 5678,
            "scenario_confirmation": NETWORK_DISCONNECT_CONFIRMATION,
            "instructions": "disconnect test",
        },
    )
    store.record_failure_event("network_disconnect", NETWORK_DISCONNECT_VERIFIED_CONFIRMATION)

    with pytest.raises(RuntimeError, match="still running"):
        store.record_network_monitor_absence(
            NETWORK_MONITOR_ABSENT_CONFIRMATION,
            process_exists_fn=lambda pid: True,
            process_start_ticks_fn=lambda pid: 5678,
        )

    entry = store.record_network_monitor_absence(
        NETWORK_MONITOR_ABSENT_CONFIRMATION,
        process_exists_fn=lambda pid: False,
        process_start_ticks_fn=lambda pid: None,
    )
    assert entry["action"] == "monitor_process_absent_after_disconnect"
    assert entry["checked_monitor_pid"] == 1234
    assert entry["process_absent"] is True


def test_network_disconnect_can_complete_when_vendor_monitor_self_exits(tmp_path):
    store = M7ReportStore(tmp_path / "network-self-exit.yaml")
    store.initialize()
    store.add_ros_capture("network_disconnect", "before", valid_ros_capture(), OBSERVATION_CONFIRMATION)
    append_live_confirmation(store, "network_disconnect")
    store.append_step(
        "network_disconnect",
        {"action": "failure_monitor_initial_snapshot", "snapshot": valid_snapshot()},
    )
    store.append_step(
        "network_disconnect",
        {
            "action": "failure_monitor_started",
            "pid": 1234,
            "process_start_ticks": 5678,
            "scenario_confirmation": NETWORK_DISCONNECT_CONFIRMATION,
            "instructions": "disconnect test",
        },
    )
    store.record_failure_event("network_disconnect", NETWORK_DISCONNECT_VERIFIED_CONFIRMATION)
    store.record_network_monitor_absence(
        NETWORK_MONITOR_ABSENT_CONFIRMATION,
        process_exists_fn=lambda pid: False,
        process_start_ticks_fn=lambda pid: None,
    )
    store.add_ros_capture("network_disconnect", "after", valid_ros_capture(), OBSERVATION_CONFIRMATION)

    store.complete_scenario(
        "network_disconnect",
        "vendor monitor exited after physical disconnect; onsite behavior recorded",
        OBSERVATION_CONFIRMATION,
        "unknown",
    )

    report = store.load()
    assert report["scenarios"]["network_disconnect"]["status"] == "COMPLETE"
    assert report["m8_permitted"] is False


def test_record_failure_event_cli_uses_disconnect_specific_confirmation(monkeypatch, tmp_path):
    report_path = tmp_path / "disconnect-event.yaml"
    store = M7ReportStore(report_path)
    store.initialize()
    store.append_step("network_disconnect", {"action": "failure_monitor_started", "pid": 1234})
    phrases = []
    monkeypatch.setattr(
        cli,
        "require_confirmation",
        lambda prompt, phrase: phrases.append(phrase) or phrase,
    )

    args = cli.build_parser().parse_args(
        ["--report", str(report_path), "record-failure-event", "--scenario", "network_disconnect"]
    )
    result = cli.command_record_failure_event(args, process_exists_fn=lambda pid: False)

    assert result == 0
    assert phrases == [NETWORK_DISCONNECT_VERIFIED_CONFIRMATION]
    actions = {step["action"] for step in store.load()["scenarios"]["network_disconnect"]["steps"]}
    assert "physical_network_disconnect_verified" in actions


def test_live_cli_without_explicit_flag_never_constructs_sdk(monkeypatch, tmp_path):
    factory = Mock(side_effect=AssertionError("SDK must not construct"))
    monkeypatch.setattr(cli, "M7SDKBoundary", lambda **kwargs: M7SDKBoundary(robot_factory=factory))

    result = cli.main(["--report", str(tmp_path / "report.yaml"), "live-read"])

    assert result == 2
    factory.assert_not_called()


def test_next_step_is_read_only_and_points_to_first_pending_scenario(monkeypatch, tmp_path, capsys):
    factory = Mock(side_effect=AssertionError("SDK must not construct"))
    monkeypatch.setattr(cli, "M7SDKBoundary", lambda **kwargs: M7SDKBoundary(robot_factory=factory))
    report_path = tmp_path / "next.yaml"
    M7ReportStore(report_path).initialize()

    result = cli.main(["--report", str(report_path), "next-step"])

    output = yaml.safe_load(capsys.readouterr().out)
    assert result == 2
    assert output["next_scenario"] == "sdk_read_only"
    assert "live-read --enable-live-sdk" in output["next_command"]
    assert output["m8_permitted"] is False
    factory.assert_not_called()


def test_preflight_import_check_never_constructs_sdk(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(cli, "validate_live_environment", Mock())
    completed = Mock(returncode=1, stdout="", stderr="missing dependency")
    runner = Mock(return_value=completed)
    monkeypatch.setattr(cli.subprocess, "run", runner)

    result = cli.main(["--report", str(tmp_path / "preflight.yaml"), "preflight"])

    output = yaml.safe_load(capsys.readouterr().out)
    assert result == 2
    assert output["sdk_import_constructed_robot"] is False
    assert output["ready_for_live_confirmation"] is False
    command = runner.call_args.args[0]
    assert "Astribot(" not in command[2]
    assert "from astribot_sdk.core.astribot_api.astribot_client import Astribot" in command[2]


def test_confirmation_ctrl_c_exits_cleanly_without_sdk_construction(monkeypatch, tmp_path, capsys):
    boundary = Mock()
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))
    monkeypatch.setattr(cli, "validate_live_environment", Mock())
    monkeypatch.setattr(cli, "require_confirmation", Mock(side_effect=KeyboardInterrupt))

    result = cli.main(
        ["--report", str(tmp_path / "cancel.yaml"), "live-read", "--enable-live-sdk"]
    )

    assert result == 130
    assert "M7 AUDIT CANCELLED" in capsys.readouterr().err
    boundary.initialize.assert_not_called()


def test_next_step_does_not_repeat_sdk_initialization_while_during_capture_is_pending(tmp_path, capsys):
    report_path = tmp_path / "during-next.yaml"
    store = M7ReportStore(report_path)
    store.initialize()
    append_runtime_evidence(store, "sdk_read_only")

    result = cli.main(["--report", str(report_path), "next-step"])

    output = yaml.safe_load(capsys.readouterr().out)
    assert result == 2
    assert output["next_scenario"] == "sdk_read_only"
    assert "ros-capture --scenario sdk_read_only --phase during" in output["next_command"]
    assert "live-read" not in output["next_command"]


def test_next_step_requires_normal_exit_before_static_hold(tmp_path, capsys):
    report_path = tmp_path / "normal-first.yaml"
    store = M7ReportStore(report_path)
    store.initialize()
    prepare_scenario(store, "sdk_read_only")
    store.complete_scenario("sdk_read_only", "read-only observed", OBSERVATION_CONFIRMATION, "safe")
    append_runtime_evidence(store, "normal_exit")

    result = cli.main(["--report", str(report_path), "next-step"])

    output = yaml.safe_load(capsys.readouterr().out)
    assert result == 2
    assert output["next_scenario"] == "normal_exit"
    assert "ros-capture --scenario normal_exit --phase after" in output["next_command"]
    assert "static-hold" not in output["next_command"]


def test_next_step_does_not_trust_capture_valid_flag_without_commands(tmp_path, capsys):
    report_path = tmp_path / "weak-next.yaml"
    store = M7ReportStore(report_path)
    store.initialize()
    append_runtime_evidence(store, "sdk_read_only")
    store.add_ros_capture(
        "sdk_read_only",
        "during",
        {"valid": True, "commands": []},
        OBSERVATION_CONFIRMATION,
    )

    result = cli.main(["--report", str(report_path), "next-step"])

    output = yaml.safe_load(capsys.readouterr().out)
    assert result == 2
    assert "ros-capture --scenario sdk_read_only --phase during" in output["next_command"]


def test_static_hold_cli_requires_both_flags_before_sdk_construction(monkeypatch, tmp_path):
    boundary = Mock()
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))

    result = cli.main(
        [
            "--report",
            str(tmp_path / "report.yaml"),
            "static-hold",
            "--enable-live-sdk",
        ]
    )

    assert result == 2
    boundary.initialize.assert_not_called()


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_monitor_rejects_invalid_snapshot_interval_before_sdk_construction(monkeypatch, tmp_path, value):
    boundary = Mock()
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))

    result = cli.main(
        [
            "--report",
            str(tmp_path / "monitor.yaml"),
            "monitor",
            "--scenario",
            "ctrl_c",
            "--enable-live-sdk",
            "--snapshot-interval-sec",
            value,
        ]
    )

    assert result == 2
    boundary.initialize.assert_not_called()


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_ros_capture_rejects_invalid_timeout_before_confirmation(monkeypatch, tmp_path, value):
    confirmation = Mock(side_effect=AssertionError("must reject before prompt"))
    monkeypatch.setattr(cli, "require_confirmation", confirmation)

    result = cli.main(
        [
            "--report",
            str(tmp_path / "capture.yaml"),
            "ros-capture",
            "--scenario",
            "sdk_read_only",
            "--phase",
            "during",
            "--timeout-sec",
            value,
        ]
    )

    assert result == 2
    confirmation.assert_not_called()


def test_live_read_cli_records_snapshot_and_normal_shutdown_with_mock(monkeypatch, tmp_path):
    report_path = tmp_path / "report.yaml"
    store = M7ReportStore(report_path)
    store.initialize({"source_sha256": {"mock": "hash"}})
    boundary = Mock()
    boundary.snapshot.return_value = valid_snapshot()
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))
    monkeypatch.setattr(cli, "validate_live_environment", Mock())
    monkeypatch.setattr(cli, "require_confirmation", lambda *args, **kwargs: args[1])

    result = cli.main(
        ["--report", str(report_path), "live-read", "--enable-live-sdk"]
    )

    assert result == 0
    boundary.initialize.assert_called_once_with()
    boundary.snapshot.assert_called_once_with()
    boundary.shutdown.assert_called_once_with()
    report = store.load()
    assert any(step["action"] == "live_sdk_read_only_snapshot" for step in report["scenarios"]["sdk_read_only"]["steps"])
    assert any(step["action"] == "sdk_shutdown_returned" for step in report["scenarios"]["normal_exit"]["steps"])


def test_static_hold_cli_records_exact_command_with_mock(monkeypatch, tmp_path):
    report_path = tmp_path / "report.yaml"
    store = M7ReportStore(report_path)
    store.initialize({"source_sha256": {"mock": "hash"}})
    boundary = Mock()
    boundary.snapshot.return_value = valid_snapshot()
    boundary.send_exact_desired_hold.return_value = {
        "before": valid_snapshot(),
        "sent_names": ["astribot_arm_left", "astribot_arm_right"],
        "exactly_equal_to_final_desired": True,
        "sent_pose": [LEFT, RIGHT],
        "final_control_rights": True,
        "final_robot_alive": True,
        "final_robot_mode": "safe",
        "max_desired_current_error_m": 0.02,
        "command_count": 1,
    }
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))
    monkeypatch.setattr(cli, "validate_live_environment", Mock())
    monkeypatch.setattr(cli, "require_confirmation", lambda *args, **kwargs: args[1])

    result = cli.main(
        [
            "--report",
            str(report_path),
            "static-hold",
            "--enable-live-sdk",
            "--enable-static-hold",
        ]
    )

    assert result == 0
    boundary.send_exact_desired_hold.assert_called_once_with(0.02)
    boundary.shutdown.assert_called_once_with()
    report = store.load()
    assert any(step["action"] == "exact_desired_static_hold" for step in report["scenarios"]["static_hold"]["steps"])


def test_process_kill_monitor_requires_live_safe_preflight_and_specific_confirmation(monkeypatch, tmp_path):
    report_path = tmp_path / "kill.yaml"
    M7ReportStore(report_path).initialize({"source_sha256": {"mock": "hash"}})
    boundary = Mock()
    boundary.snapshot.return_value = {"control_rights": True, "robot_alive": True, "robot_mode": "safe"}
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))
    monkeypatch.setattr(cli, "validate_live_environment", Mock())
    confirmations = []

    def confirm(prompt, phrase):
        confirmations.append(phrase)
        if phrase == cli.PROCESS_KILL_CONFIRMATION:
            raise RuntimeError("stop before exposing kill instructions")
        return phrase

    monkeypatch.setattr(cli, "require_confirmation", confirm)

    result = cli.main(
        [
            "--report",
            str(report_path),
            "monitor",
            "--scenario",
            "process_kill",
            "--enable-live-sdk",
        ]
    )

    assert result == 2
    assert confirmations == [cli.LIVE_CONFIRMATION, cli.PROCESS_KILL_CONFIRMATION]
    boundary.shutdown.assert_called_once_with()
    report = M7ReportStore(report_path).load()
    actions = {step["action"] for step in report["scenarios"]["process_kill"]["steps"]}
    assert "failure_monitor_initial_snapshot" in actions
    assert "failure_monitor_started" not in actions


def test_ctrl_c_signal_evidence_is_yaml_serializable(tmp_path):
    store = M7ReportStore(tmp_path / "ctrl-c.yaml")

    entry = store.append_step(
        "ctrl_c",
        {"action": "keyboard_interrupt_received", "signal": int(signal.SIGINT)},
    )

    assert entry["signal"] == 2
    assert yaml.safe_load(store.path.read_text(encoding="utf-8"))["scenarios"]["ctrl_c"]["steps"][0]["signal"] == 2


def test_ctrl_c_monitor_cli_records_signal_and_shutdown_end_to_end(monkeypatch, tmp_path):
    report_path = tmp_path / "ctrl-c-cli.yaml"
    store = M7ReportStore(report_path)
    store.initialize({"source_sha256": {"mock": "hash"}})
    boundary = Mock()
    boundary.snapshot.return_value = valid_snapshot()
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))
    monkeypatch.setattr(cli, "validate_live_environment", Mock())
    monkeypatch.setattr(cli, "require_confirmation", lambda prompt, phrase: phrase)
    install_signal_handler = Mock()
    monkeypatch.setattr(cli.signal, "signal", install_signal_handler)
    monkeypatch.setattr(cli.time, "monotonic", Mock(side_effect=[0.0, 0.0]))
    monkeypatch.setattr(cli.time, "sleep", Mock(side_effect=KeyboardInterrupt))

    result = cli.main(
        [
            "--report",
            str(report_path),
            "monitor",
            "--scenario",
            "ctrl_c",
            "--enable-live-sdk",
        ]
    )

    assert result == 0
    install_signal_handler.assert_called_once_with(signal.SIGINT, signal.default_int_handler)
    boundary.shutdown.assert_called_once_with()
    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    steps = report["scenarios"]["ctrl_c"]["steps"]
    assert any(
        step.get("action") == "python_sigint_handler_restored" and step.get("signal") == 2
        for step in steps
    )
    assert any(step.get("action") == "keyboard_interrupt_received" and step.get("signal") == 2 for step in steps)
    assert any(step.get("action") == "sdk_shutdown_returned_after_monitor" and step.get("success") is True for step in steps)


@pytest.mark.parametrize(
    "snapshot, message",
    [
        ({"control_rights": False, "robot_alive": True, "robot_mode": "safe"}, "control rights"),
        ({"control_rights": True, "robot_alive": False, "robot_mode": "safe"}, "live robot"),
        ({"control_rights": True, "robot_alive": True, "robot_mode": "professional"}, "safe mode"),
    ],
)
def test_failure_monitor_fails_closed_before_manual_action(monkeypatch, tmp_path, snapshot, message):
    boundary = Mock()
    boundary.snapshot.return_value = snapshot
    monkeypatch.setattr(cli, "M7SDKBoundary", Mock(return_value=boundary))
    monkeypatch.setattr(cli, "validate_live_environment", Mock())
    monkeypatch.setattr(cli, "require_confirmation", lambda prompt, phrase: phrase)

    result = cli.main(
        [
            "--report",
            str(tmp_path / "monitor.yaml"),
            "monitor",
            "--scenario",
            "network_disconnect",
            "--enable-live-sdk",
        ]
    )

    assert result == 2
    boundary.shutdown.assert_called_once_with()
    actions = {
        step["action"]
        for step in M7ReportStore(tmp_path / "monitor.yaml").load()["scenarios"]["network_disconnect"]["steps"]
    }
    assert "failure_monitor_started" not in actions


def test_live_environment_must_match_robot_network_and_static_hashes(monkeypatch, tmp_path):
    store = M7ReportStore(tmp_path / "report.yaml")
    store.initialize(inspect_sdk_source("/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master"))
    monkeypatch.setenv("ROS_DOMAIN_ID", "24")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setenv("ROBOT_TYPE", "S1")

    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        validate_live_environment(store, "/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master")


def test_live_environment_accepts_expected_values_and_matching_hashes(monkeypatch, tmp_path):
    root = "/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master"
    store = M7ReportStore(tmp_path / "report.yaml")
    store.initialize(inspect_sdk_source(root))
    monkeypatch.setenv("ROS_DOMAIN_ID", "25")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setenv("ROBOT_TYPE", "S1")

    findings = validate_live_environment(store, root)

    assert findings["source_sha256"] == store.load()["static_sdk_findings"]["source_sha256"]


def test_ros_capture_uses_read_only_ros2_commands_only():
    calls = []

    def runner(command, **kwargs):
        calls.append(command)
        if command == ["ros2", "topic", "list", "-t"]:
            stdout = "/astribot/heartbeat [std_msgs/msg/Int32MultiArray]\n"
        elif command == ["ros2", "service", "list", "-t"]:
            stdout = "/astribot/control_rights [astribot_msgs/srv/RawRequest]\n"
        else:
            stdout = "ok"
        return Mock(returncode=0, stdout=stdout, stderr="")

    capture = capture_ros_read_only(runner=runner)

    assert capture["commands"]
    assert all(command[:2] in (["ros2", "node"], ["ros2", "service"], ["ros2", "topic"]) for command in calls)
    assert not any("pub" in command or "service call" in " ".join(command) for command in calls)
    assert capture["heartbeat_topics"] == ["/astribot/heartbeat"]
    assert ["ros2", "topic", "info", "/astribot/heartbeat", "--verbose"] in calls
    assert capture["valid"] is True
    assert capture["control_rights_service_present"] is True


def test_ros_capture_can_be_valid_when_control_rights_service_is_absent_after_shutdown():
    def runner(command, **kwargs):
        if command == ["ros2", "service", "list", "-t"]:
            return Mock(returncode=0, stdout="/other_service [std_srvs/srv/Trigger]\n", stderr="")
        if command == ["ros2", "service", "type", "/astribot/control_rights"]:
            return Mock(returncode=1, stdout="", stderr="Unknown service")
        return Mock(returncode=0, stdout="captured", stderr="")

    capture = capture_ros_read_only(runner=runner)

    assert capture["valid"] is True
    assert capture["control_rights_service_present"] is False


def test_ros_capture_cli_returns_nonzero_when_required_topics_are_missing(monkeypatch, tmp_path):
    monkeypatch.setattr(cli, "require_confirmation", lambda *args, **kwargs: OBSERVATION_CONFIRMATION)
    monkeypatch.setattr(
        cli,
        "capture_ros_read_only",
        lambda **kwargs: {"valid": False, "validation_failures": ["missing endpoint"], "commands": []},
    )

    result = cli.main(
        [
            "--report",
            str(tmp_path / "report.yaml"),
            "ros-capture",
            "--scenario",
            "sdk_read_only",
            "--phase",
            "during",
        ]
    )

    assert result == 2


def test_report_is_machine_readable_yaml(tmp_path):
    path = tmp_path / "report.yaml"
    store = M7ReportStore(path)
    store.initialize({"heartbeat": "0.1 sec"})

    parsed = yaml.safe_load(path.read_text())

    assert parsed["schema"] == "astribot_s1_m7_hardware_audit.v1"
    assert parsed["status"] == "INCOMPLETE"

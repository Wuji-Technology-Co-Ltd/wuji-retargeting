import time
import json
from pathlib import Path

import yaml

from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_angle_xyzw, quat_from_yaw_y_up
from stardust_wuji_quest3_pc_retargeting.sim.mock_webxr_sender import build_mock_frame
from stardust_wuji_quest3_pc_retargeting.runtime.supervisor import ControlPCSupervisor
from stardust_wuji_quest3_pc_retargeting.arm_control.astribot_adapter import AstribotAdapter
from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmTarget
from stardust_wuji_quest3_pc_retargeting.protocol.validation import validate_tracking_frame
import pytest


def config():
    data = yaml.safe_load((Path(__file__).parents[1] / "configs/arm/s1_quest3_default.yaml").read_text())
    data["absolute_session_calibration"].update(
        {"countdown_sec": 0.0, "sample_duration_sec": 0.03, "minimum_valid_samples": 3}
    )
    data["safety"].update(
        {
            "engage_stable_frames": 1,
            "engage_hold_duration_sec": 0.0,
            "engage_soft_start_duration_sec": 0.0,
        }
    )
    return data


def feed(supervisor, seq, **mutations):
    frame = build_mock_frame(seq)
    for key, value in mutations.items():
        if key == "session_active":
            frame["session"]["active"] = value
        elif key == "left_valid":
            frame["hands"]["left"]["valid"] = value
        elif key == "right_valid":
            frame["hands"]["right"]["valid"] = value
        elif key == "right_wrist_x_offset":
            wrist_index = frame["hands"]["right"]["joint_names"].index("wrist")
            frame["hands"]["right"]["positions"][wrist_index][0] += value
        elif key == "right_wrist_yaw_rad":
            wrist_index = frame["hands"]["right"]["joint_names"].index("wrist")
            frame["hands"]["right"]["orientations_xyzw"][wrist_index] = quat_from_yaw_y_up(value).tolist()
        elif key == "both_wrist_z":
            for side in ("left", "right"):
                wrist_index = frame["hands"][side]["joint_names"].index("wrist")
                frame["hands"][side]["positions"][wrist_index][2] = value
        elif key == "hmd_yaw_rad":
            frame["hmd"]["orientation_xyzw"] = quat_from_yaw_y_up(value).tolist()
        elif key == "revision":
            frame["session"]["reference_space_revision"] = value
    supervisor.ingest_payload(frame)
    return frame


def test_relative_requires_recenter_and_pause_requires_new_recenter():
    supervisor = ControlPCSupervisor(config(), arm="left")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("start").accepted is False
        assert supervisor.execute_command("recenter").accepted is True
        assert supervisor.status_snapshot().teleop_state == "ARMED"
        assert supervisor.adapter.stats.send_calls == 0
        assert supervisor.execute_command("start").accepted is True
        assert supervisor.status_snapshot().teleop_state == "RUNNING"
        assert supervisor.execute_command("pause").accepted is True
        assert supervisor.execute_command("start").accepted is False
        feed(supervisor, 2)
        assert supervisor.execute_command("recenter").accepted is True
    finally:
        supervisor.close()


def test_relative_engage_atomically_recenters_and_starts():
    supervisor = ControlPCSupervisor(config(), arm="left")
    supervisor.start()
    try:
        feed(supervisor, 1)
        result = supervisor.execute_command("engage")
        assert result.accepted is True
        assert "auto-start" in result.message
        assert supervisor.status_snapshot().teleop_state == "RUNNING"
        assert supervisor.mapper.is_calibrated("left")
    finally:
        supervisor.close()


def test_relative_engage_stabilizes_hmd_and_wrists_then_soft_starts():
    cfg = config()
    cfg["safety"].update(
        {
            "engage_stable_frames": 3,
            "engage_hold_duration_sec": 0.02,
            "engage_soft_start_duration_sec": 0.04,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1, hmd_yaw_rad=0.5, both_wrist_z=0.0)
        result = supervisor.execute_command("engage")
        assert result.accepted is True
        assert supervisor.status_snapshot().engage_state == "STABILIZING"
        assert supervisor.status_snapshot().teleop_state == "PAUSED"

        feed(supervisor, 2, hmd_yaw_rad=0.5, both_wrist_z=0.0)
        time.sleep(0.01)
        feed(supervisor, 3, hmd_yaw_rad=0.5, both_wrist_z=0.0)
        deadline = time.monotonic() + 1.0
        while supervisor.status_snapshot().engage_state != "SOFT_START" and time.monotonic() < deadline:
            time.sleep(0.005)
        status = supervisor.status_snapshot()
        assert status.engage_state == "SOFT_START"
        assert status.engage_operator_yaw_rad == pytest.approx(0.5, abs=1e-6)
        assert status.teleop_state == "PAUSED"

        time.sleep(0.03)
        feed(supervisor, 4, hmd_yaw_rad=0.5, both_wrist_z=0.0, right_wrist_x_offset=0.01)
        time.sleep(0.05)
        feed(supervisor, 5, hmd_yaw_rad=0.5, both_wrist_z=0.0, right_wrist_x_offset=0.01)
        deadline = time.monotonic() + 1.0
        while supervisor.status_snapshot().teleop_state != "RUNNING" and time.monotonic() < deadline:
            time.sleep(0.005)
        status = supervisor.status_snapshot()
        assert status.engage_state == "IDLE"
        assert status.teleop_state == "RUNNING"
        assert supervisor.adapter.stats.send_calls > 0
    finally:
        supervisor.close()


def test_relative_engage_requires_stable_multiframe_window():
    cfg = config()
    cfg["safety"].update(
        {
            "engage_stable_frames": 3,
            "engage_wrist_position_stability_m": 0.005,
            "engage_hold_duration_sec": 0.0,
            "engage_soft_start_duration_sec": 0.0,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("engage").accepted is True
        for seq, offset in ((2, 0.03), (3, -0.03), (4, 0.03), (5, -0.03)):
            feed(supervisor, seq, right_wrist_x_offset=offset)
            time.sleep(0.01)
        status = supervisor.status_snapshot()
        assert status.engage_state == "STABILIZING"
        assert status.teleop_state == "PAUSED"
        assert supervisor.adapter.stats.send_calls == 0
    finally:
        supervisor.close()


@pytest.mark.parametrize("fixed_anchor_mode", [False, True])
def test_relative_engage_anchors_to_current_cartesian_pose(fixed_anchor_mode):
    cfg = config()
    cfg["safety"]["fixed_anchor_mode"] = fixed_anchor_mode
    supervisor = ControlPCSupervisor(cfg, arm="left")
    supervisor.start()
    try:
        supervisor.wait_until_adapter_ready(1.0)
        desired = supervisor.adapter.get_desired_poses()
        current = supervisor.adapter.get_current_poses()
        current["left"] = ArmTarget(
            [desired["left"].position[0] - 0.02, desired["left"].position[1], desired["left"].position[2]],
            desired["left"].orientation_xyzw,
        )
        supervisor.adapter.set_dryrun_poses(desired=desired, current=current)
        feed(supervisor, 1)
        result = supervisor.execute_command("engage")
        assert result.accepted is True
        anchor = supervisor.mapper._relative["left"].robot
        assert anchor.position == pytest.approx(current["left"].position)
        assert anchor.orientation_xyzw == pytest.approx(current["left"].orientation_xyzw)
        assert supervisor.arm_filters["left"]._last_target.position == pytest.approx(current["left"].position)
    finally:
        supervisor.close()


def test_dual_relative_engage_anchors_and_commands_both_arms_as_one_cycle():
    supervisor = ControlPCSupervisor(config(), arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        result = supervisor.execute_command("engage")
        assert result.accepted is True
        assert supervisor.mapper.is_calibrated("left")
        assert supervisor.mapper.is_calibrated("right")
        feed(supervisor, 2)
        deadline = time.monotonic() + 1.0
        while set(supervisor.adapter.last_targets) != {"left", "right"} and time.monotonic() < deadline:
            time.sleep(0.01)
        assert set(supervisor.adapter.last_targets) == {"left", "right"}
    finally:
        supervisor.close()


def test_dual_relative_latches_pause_when_either_hand_loses_tracking():
    supervisor = ControlPCSupervisor(config(), arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("engage").accepted is True
        feed(supervisor, 2, right_valid=False)
        deadline = time.monotonic() + 1.0
        while supervisor.status_snapshot().loop_state != "PAUSED" and time.monotonic() < deadline:
            time.sleep(0.01)
        status = supervisor.status_snapshot()
        assert status.loop_state == "PAUSED"
        assert status.teleop_state == "PAUSED"
        assert "dual-arm tracking invalid: right" in supervisor.loop.fault_reason
    finally:
        supervisor.close()


def test_dual_relative_reacquires_with_reanchor_and_absorbs_large_reentry_offset():
    cfg = config()
    cfg["safety"]["hand_reacquire_timeout_sec"] = 5.0
    cfg["safety"]["hand_reacquire_stable_frames"] = 3
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("engage").accepted is True
        before = supervisor.adapter.get_desired_poses()["right"]
        feed(supervisor, 2, right_valid=False)
        deadline = time.monotonic() + 1.0
        while supervisor.status_snapshot().hand_reacquire_state != "WAITING" and time.monotonic() < deadline:
            time.sleep(0.01)
        assert supervisor.status_snapshot().loop_state == "PAUSED"

        for seq in (3, 4, 5, 6):
            feed(supervisor, seq, right_wrist_x_offset=0.25)
            time.sleep(0.02)
        status = supervisor.status_snapshot()
        assert status.teleop_state == "RUNNING", status.last_error
        assert status.hand_reacquire_state == "IDLE"
        assert supervisor.loop.pause_latched is False
        after = supervisor.adapter.last_targets["right"]
        assert after.position == pytest.approx(before.position, abs=0.005)
        assert after.orientation_xyzw == pytest.approx(before.orientation_xyzw, abs=1e-9)
    finally:
        supervisor.close()


def test_dual_reacquire_holds_position_and_catches_up_to_absolute_hand_orientation():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["filter"]["orientation_alpha"] = 1.0
    cfg["safety"].update(
        {
            "hand_reacquire_timeout_sec": 5.0,
            "hand_reacquire_stable_frames": 3,
            "absolute_orientation_reacquire": True,
            "orientation_reacquire_speed_rad_s": 10.0,
            "orientation_reacquire_direct_error_rad": 0.15,
            "orientation_reacquire_complete_error_rad": 0.087,
            "orientation_reacquire_max_error_rad": 1.57,
            "orientation_reacquire_complete_frames": 2,
            "max_angular_speed_rad_s": 20.0,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("engage").accepted is True
        held_position = supervisor.adapter.get_desired_poses()["right"].position
        feed(supervisor, 2, right_valid=False)
        time.sleep(0.02)

        saw_catchup = False
        for seq in range(3, 30):
            feed(supervisor, seq, right_wrist_x_offset=0.25, right_wrist_yaw_rad=1.0)
            time.sleep(0.015)
            status = supervisor.status_snapshot()
            saw_catchup = saw_catchup or status.hand_reacquire_state == "ORIENTATION_CATCHUP"
            if saw_catchup and status.hand_reacquire_state == "IDLE":
                break
        status = supervisor.status_snapshot()
        assert saw_catchup is True
        assert status.hand_reacquire_state == "IDLE"
        assert status.teleop_state == "RUNNING"
        target = supervisor.adapter.last_targets["right"]
        assert target.position == pytest.approx(held_position, abs=0.01)
        assert quat_angle_xyzw(target.orientation_xyzw, [0, 0, 0, 1]) == pytest.approx(1.0, abs=0.10)
    finally:
        supervisor.close()


def test_dual_reacquire_requires_alignment_above_automatic_orientation_error():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["safety"].update(
        {
            "hand_reacquire_timeout_sec": 5.0,
            "hand_reacquire_stable_frames": 2,
            "absolute_orientation_reacquire": True,
            "orientation_reacquire_speed_rad_s": 1.0,
            "orientation_reacquire_direct_error_rad": 0.15,
            "orientation_reacquire_complete_error_rad": 0.087,
            "orientation_reacquire_max_error_rad": 1.57,
            "orientation_reacquire_complete_frames": 2,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("engage").accepted is True
        feed(supervisor, 2, right_valid=False)
        time.sleep(0.02)
        for seq in (3, 4, 5):
            feed(supervisor, seq, right_wrist_yaw_rad=2.0)
            time.sleep(0.02)
        status = supervisor.status_snapshot()
        assert status.hand_reacquire_state == "ALIGNMENT_REQUIRED"
        assert status.teleop_state == "PAUSED"
        assert status.reacquire_orientation_errors_rad["right"] > 1.57
    finally:
        supervisor.close()


def test_fixed_anchor_calibrate_start_and_position_clutch_command_contract():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["safety"]["fixed_anchor_mode"] = True
    cfg["safety"]["fixed_anchor_pose_reacquire"] = True
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        calibrated = supervisor.execute_command("anchor-calibrate")
        assert calibrated.accepted is True
        assert supervisor.status_snapshot().teleop_state == "ARMED"
        assert supervisor.adapter.stats.send_calls == 0
        assert supervisor.execute_command("start").accepted is True
        assert supervisor.execute_command("pause").accepted is True
        feed(supervisor, 2, right_wrist_x_offset=0.08, right_wrist_yaw_rad=0.10)
        clutch = supervisor.execute_command("clutch-position")
        assert clutch.accepted is True
        assert "orientation anchor preserved" in clutch.message
        assert supervisor.status_snapshot().teleop_state == "ARMED"
    finally:
        supervisor.close()


def test_fixed_anchor_atomic_engage_and_clutch_resume_command_contract():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["safety"]["fixed_anchor_mode"] = True
    cfg["safety"]["fixed_anchor_pose_reacquire"] = True
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        engaged = supervisor.execute_command("engage")
        assert engaged.accepted is True
        assert "auto-start" in engaged.message
        assert supervisor.status_snapshot().teleop_state == "RUNNING"

        assert supervisor.execute_command("pause").accepted is True
        feed(supervisor, 2, right_wrist_x_offset=0.08, right_wrist_yaw_rad=0.10)
        original_orientation_anchor = supervisor.mapper._relative["right"].robot.orientation_array()
        resumed = supervisor.execute_command("clutch-resume")
        assert resumed.accepted is True
        assert "atomically" in resumed.message
        assert supervisor.status_snapshot().teleop_state == "RUNNING"
        assert supervisor.mapper._relative["right"].robot.orientation_array() == pytest.approx(
            original_orientation_anchor
        )
    finally:
        supervisor.close()


def test_fixed_anchor_clutch_resume_uses_bounded_orientation_catchup_above_start_gate():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["filter"]["position_alpha"] = 1.0
    cfg["filter"]["orientation_alpha"] = 1.0
    cfg["safety"].update(
        {
            "fixed_anchor_mode": True,
            "fixed_anchor_pose_reacquire": True,
            "max_linear_speed_mps": 20.0,
            "max_angular_speed_rad_s": 20.0,
            "absolute_reacquire_linear_speed_mps": 10.0,
            "absolute_reacquire_direct_position_error_m": 0.002,
            "absolute_reacquire_complete_position_error_m": 0.002,
            "absolute_reacquire_max_position_error_m": 0.30,
            "pose_reacquire_linear_accel_mps2": 1000.0,
            "pose_reacquire_angular_accel_rad_s2": 1000.0,
            "orientation_reacquire_speed_rad_s": 10.0,
            "orientation_reacquire_direct_error_rad": 0.02,
            "orientation_reacquire_complete_error_rad": 0.02,
            "orientation_reacquire_max_error_rad": 1.57,
            "orientation_reacquire_complete_frames": 2,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("anchor-engage").accepted is True
        assert supervisor.execute_command("pause").accepted is True
        feed(supervisor, 2, right_wrist_yaw_rad=0.60, both_wrist_z=0.01)

        result = supervisor.execute_command("clutch-resume")

        assert result.accepted is True
        assert "orientation catch-up started" in result.message
        assert supervisor.status_snapshot().hand_reacquire_state == "FIXED_ANCHOR_CATCHUP"
        assert "0.6000 rad" in supervisor.status_snapshot().hand_reacquire_trigger_reason

        saw_catchup = False
        for seq in range(3, 35):
            feed(supervisor, seq, right_wrist_yaw_rad=0.60, both_wrist_z=0.01)
            time.sleep(0.015)
            state = supervisor.status_snapshot().hand_reacquire_state
            saw_catchup = saw_catchup or state == "FIXED_ANCHOR_CATCHUP"
            if saw_catchup and state == "IDLE":
                break
        assert saw_catchup is True
        assert supervisor.status_snapshot().hand_reacquire_state == "IDLE"
        assert supervisor.status_snapshot().hand_tracking_recovery_completions == 1
    finally:
        supervisor.close()


def test_fixed_anchor_recover_init_pauses_atomically_then_stops_until_new_engage():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["safety"]["fixed_anchor_mode"] = True
    cfg["safety"]["fixed_anchor_pose_reacquire"] = True
    cfg["safety"]["init_recovery_enabled"] = True
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("anchor-engage").accepted is True
        feed(supervisor, 2, both_wrist_z=0.01)
        recovered = supervisor.execute_command("recover-init", timeout=5.0)

        assert recovered.accepted is True
        assert "engage required" in recovered.message
        assert supervisor.status_snapshot().teleop_state == "PAUSED"
        assert supervisor.adapter.stats.init_recovery_calls == 1
        assert supervisor.mapper.is_calibrated("left") is False
        assert supervisor.mapper.is_calibrated("right") is False

        feed(supervisor, 3, both_wrist_z=0.01)
        engaged = supervisor.execute_command("engage")
        assert engaged.accepted is True
        assert supervisor.status_snapshot().teleop_state == "RUNNING"
    finally:
        supervisor.close()


def test_fixed_anchor_arm_uses_wrist_when_full_hand_joints_are_invalid():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["safety"]["fixed_anchor_mode"] = True
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        payload = build_mock_frame(1)
        payload["arm_wrists"] = {}
        for side in ("left", "right"):
            hand = payload["hands"][side]
            wrist_index = hand["joint_names"].index("wrist")
            payload["arm_wrists"][side] = {
                "valid": True,
                "position": list(hand["positions"][wrist_index]),
                "orientation_xyzw": list(hand["orientations_xyzw"][wrist_index]),
            }
        payload["hands"]["left"] = {
            "valid": False,
            "joint_names": [],
            "positions": [],
            "orientations_xyzw": [],
        }
        supervisor.ingest_payload(payload)

        assert supervisor.execute_command("anchor-engage").accepted is True
        status = supervisor.status_snapshot()
        assert status.hand_tracking["left"] is False
        assert status.arm_wrist_tracking["left"] is True
        assert status.teleop_state == "RUNNING"
        assert status.hand_reacquire_state == "IDLE"
    finally:
        supervisor.close()


def test_fixed_anchor_reacquire_catches_up_without_replacing_original_anchor():
    cfg = config()
    cfg["mapping"]["position_scale_xyz"] = [1.0, 1.0, 1.0]
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["filter"]["position_alpha"] = 1.0
    cfg["filter"]["orientation_alpha"] = 1.0
    cfg["safety"].update(
        {
            "fixed_anchor_mode": True,
            "fixed_anchor_pose_reacquire": True,
            "max_linear_speed_mps": 20.0,
            "max_angular_speed_rad_s": 20.0,
            "hand_reacquire_timeout_sec": 5.0,
            "hand_reacquire_stable_frames": 3,
            "absolute_reacquire_linear_speed_mps": 10.0,
            "absolute_reacquire_direct_position_error_m": 0.02,
            "absolute_reacquire_complete_position_error_m": 0.01,
            "absolute_reacquire_max_position_error_m": 0.30,
            "pose_reacquire_linear_accel_mps2": 1000.0,
            "pose_reacquire_angular_accel_rad_s2": 1000.0,
            "orientation_reacquire_speed_rad_s": 10.0,
            "orientation_reacquire_direct_error_rad": 0.15,
            "orientation_reacquire_complete_error_rad": 0.087,
            "orientation_reacquire_max_error_rad": 1.57,
            "orientation_reacquire_complete_frames": 2,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("anchor-calibrate").accepted is True
        initial_right = supervisor.adapter.get_desired_poses()["right"]
        assert supervisor.execute_command("start").accepted is True
        feed(supervisor, 2, right_valid=False)
        time.sleep(0.02)

        saw_catchup = False
        for seq in range(3, 35):
            feed(
                supervisor,
                seq,
                right_wrist_x_offset=0.10,
                right_wrist_yaw_rad=1.0,
                both_wrist_z=0.01,
            )
            time.sleep(0.015)
            state = supervisor.status_snapshot().hand_reacquire_state
            saw_catchup = saw_catchup or state == "FIXED_ANCHOR_CATCHUP"
            if saw_catchup and state == "IDLE":
                break
        assert saw_catchup is True
        assert supervisor.status_snapshot().hand_reacquire_state == "IDLE"

        neutral_frame = validate_tracking_frame(build_mock_frame(1))
        neutral = supervisor._wrist_pose(neutral_frame, "right")
        neutral_candidate = supervisor.mapper.map_hand("right", neutral)
        assert neutral_candidate.position == pytest.approx(initial_right.position, abs=1e-9)
        assert neutral_candidate.orientation_xyzw == pytest.approx(initial_right.orientation_xyzw, abs=1e-9)
    finally:
        supervisor.close()


def test_fixed_anchor_input_jump_without_invalid_frame_enters_recovery_instead_of_sticking():
    cfg = config()
    cfg["mapping"]["position_scale_xyz"] = [1.0, 1.0, 1.0]
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["filter"]["position_alpha"] = 1.0
    cfg["filter"]["orientation_alpha"] = 1.0
    cfg["safety"].update(
        {
            "fixed_anchor_mode": True,
            "fixed_anchor_pose_reacquire": True,
            "max_input_position_jump_m": 0.03,
            "max_linear_speed_mps": 20.0,
            "max_angular_speed_rad_s": 20.0,
            "hand_reacquire_timeout_sec": 5.0,
            "hand_reacquire_stable_frames": 2,
            "absolute_reacquire_linear_speed_mps": 0.20,
            "absolute_reacquire_direct_position_error_m": 0.002,
            "absolute_reacquire_complete_position_error_m": 0.002,
            "absolute_reacquire_max_position_error_m": 0.30,
            "pose_reacquire_linear_accel_mps2": 1000.0,
            "pose_reacquire_angular_accel_rad_s2": 1000.0,
            "orientation_reacquire_speed_rad_s": 10.0,
            "orientation_reacquire_direct_error_rad": 0.02,
            "orientation_reacquire_complete_error_rad": 0.02,
            "orientation_reacquire_max_error_rad": 1.57,
            "orientation_reacquire_complete_frames": 2,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("anchor-engage").accepted is True

        feed(supervisor, 2, right_wrist_x_offset=0.10, both_wrist_z=0.01)
        deadline = time.monotonic() + 1.0
        while supervisor.status_snapshot().hand_reacquire_state == "IDLE" and time.monotonic() < deadline:
            time.sleep(0.01)
        status = supervisor.status_snapshot()
        assert status.hand_reacquire_state in {"WAITING", "STABILIZING"}
        assert status.hand_tracking_loss_events == 1
        assert "input position jump" in status.last_arm_filter_rejections["right"]
        assert "right" in status.hand_reacquire_trigger_reason

        saw_catchup = False
        for seq in range(3, 110):
            feed(supervisor, seq, right_wrist_x_offset=0.10, both_wrist_z=0.01)
            time.sleep(0.015)
            state = supervisor.status_snapshot().hand_reacquire_state
            saw_catchup = saw_catchup or state == "FIXED_ANCHOR_CATCHUP"
            if saw_catchup and state == "IDLE":
                break
        assert saw_catchup is True
        assert supervisor.status_snapshot().hand_reacquire_state == "IDLE"
        assert supervisor.status_snapshot().hand_tracking_recovery_completions == 1
    finally:
        supervisor.close()


def test_fixed_anchor_reference_space_change_requires_recalibration():
    cfg = config()
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["safety"]["fixed_anchor_mode"] = True
    cfg["safety"]["fixed_anchor_pose_reacquire"] = True
    supervisor = ControlPCSupervisor(cfg, arm="both")
    supervisor.start()
    try:
        feed(supervisor, 1, revision=0)
        assert supervisor.execute_command("engage").accepted is True
        feed(supervisor, 2, revision=1)
        time.sleep(0.03)
        assert supervisor.execute_command("pause").accepted is True
        feed(supervisor, 3, revision=1)
        reengaged = supervisor.execute_command("engage")
        assert reengaged.accepted is True
        assert supervisor.status_snapshot().teleop_state == "RUNNING"
    finally:
        supervisor.close()


def test_absolute_calibration_finishes_armed_without_sending_and_writes_report(tmp_path):
    report = tmp_path / "absolute.yaml"
    supervisor = ControlPCSupervisor(config(), arm="left", absolute_calibration_report=report)
    supervisor.start()
    try:
        feed(supervisor, 1)
        result = supervisor.execute_command("absolute-calibrate")
        assert result.accepted is True
        assert supervisor.adapter.stats.send_calls == 0
        for seq in range(2, 9):
            feed(supervisor, seq)
            time.sleep(0.01)
        deadline = time.monotonic() + 1.0
        while supervisor.status_snapshot().calibration_state not in {"VALID", "INVALID"} and time.monotonic() < deadline:
            time.sleep(0.01)
        status = supervisor.status_snapshot()
        assert status.calibration_state == "VALID", status.calibration_failure_reason
        assert status.mapping_mode == "absolute"
        assert status.teleop_state == "ARMED"
        assert supervisor.adapter.stats.send_calls == 0
        assert report.exists()
        assert yaml.safe_load(report.read_text())["restorable"] is False
        feed(supervisor, 10)
        assert supervisor.execute_command("engage").accepted is True
    finally:
        supervisor.close()


def test_absolute_engage_recalibrates_and_starts_automatically():
    supervisor = ControlPCSupervisor(config(), arm="left", mapping_mode="absolute")
    supervisor.start()
    try:
        feed(supervisor, 1)
        result = supervisor.execute_command("engage")
        assert result.accepted is True
        assert "automatically" in result.message
        for seq in range(2, 20):
            feed(supervisor, seq)
            time.sleep(0.01)
            if supervisor.status_snapshot().teleop_state == "RUNNING":
                break
        status = supervisor.status_snapshot()
        assert status.calibration_state == "VALID"
        assert status.teleop_state == "RUNNING", status.last_error
    finally:
        supervisor.close()


def test_full_absolute_reacquire_catches_up_position_and_orientation():
    cfg = config()
    cfg["mapping"]["position_scale_xyz"] = [1.0, 1.0, 1.0]
    cfg["mapping"]["enable_orientation"] = True
    cfg["mapping"]["rotation_scale"] = 1.0
    cfg["filter"]["position_alpha"] = 1.0
    cfg["filter"]["orientation_alpha"] = 1.0
    cfg["safety"].update(
        {
            "max_linear_speed_mps": 20.0,
            "max_angular_speed_rad_s": 20.0,
            "hand_reacquire_timeout_sec": 5.0,
            "hand_reacquire_stable_frames": 3,
            "absolute_pose_reacquire": True,
            "absolute_reacquire_linear_speed_mps": 10.0,
            "absolute_reacquire_direct_position_error_m": 0.02,
            "absolute_reacquire_complete_position_error_m": 0.01,
            "absolute_reacquire_max_position_error_m": 0.30,
            "orientation_reacquire_speed_rad_s": 10.0,
            "orientation_reacquire_direct_error_rad": 0.15,
            "orientation_reacquire_complete_error_rad": 0.087,
            "orientation_reacquire_max_error_rad": 1.57,
            "orientation_reacquire_complete_frames": 2,
        }
    )
    supervisor = ControlPCSupervisor(cfg, arm="both", mapping_mode="absolute")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("absolute-calibrate").accepted is True
        for seq in range(2, 20):
            feed(supervisor, seq)
            time.sleep(0.01)
            if supervisor.status_snapshot().calibration_state == "VALID":
                break
        assert supervisor.status_snapshot().calibration_state == "VALID"
        feed(supervisor, 20)
        assert supervisor.execute_command("start").accepted is True
        held = supervisor.adapter.get_desired_poses()["right"]

        feed(supervisor, 21, right_valid=False)
        time.sleep(0.02)
        saw_catchup = False
        for seq in range(22, 60):
            feed(
                supervisor,
                seq,
                right_wrist_x_offset=0.10,
                right_wrist_yaw_rad=1.0,
                both_wrist_z=0.0,
            )
            time.sleep(0.015)
            state = supervisor.status_snapshot().hand_reacquire_state
            saw_catchup = saw_catchup or state == "ABSOLUTE_POSE_CATCHUP"
            if saw_catchup and state == "IDLE":
                break
        status = supervisor.status_snapshot()
        assert saw_catchup is True
        assert status.hand_reacquire_state == "IDLE"
        assert status.teleop_state == "RUNNING"
        target = supervisor.adapter.last_targets["right"]
        assert abs(target.position[1] - held.position[1]) > 0.05
        assert quat_angle_xyzw(target.orientation_xyzw, held.orientation_xyzw) > 0.5
    finally:
        supervisor.close()


def test_running_mode_switch_rejected_and_session_revision_invalidates_absolute():
    supervisor = ControlPCSupervisor(config(), arm="left")
    supervisor.start()
    try:
        feed(supervisor, 1)
        supervisor.execute_command("recenter")
        supervisor.execute_command("start")
        rejected = supervisor.execute_command("mode", "absolute")
        assert rejected.accepted is False
        assert supervisor.mapper.mode.value == "relative"
        assert "RUNNING" in rejected.message
    finally:
        supervisor.close()


def test_tracking_loss_and_disconnect_pause_without_new_arm_output():
    supervisor = ControlPCSupervisor(config(), arm="left")
    supervisor.start()
    try:
        feed(supervisor, 1)
        supervisor.execute_command("recenter")
        supervisor.execute_command("start")
        feed(supervisor, 2)
        time.sleep(0.03)
        sent_before_loss = supervisor.adapter.stats.send_calls
        feed(supervisor, 3, left_valid=False)
        time.sleep(0.03)
        assert supervisor.adapter.stats.send_calls >= sent_before_loss
        time.sleep(0.12)
        assert supervisor.loop.state.value == "PAUSED"
    finally:
        supervisor.close()


def test_default_supervisor_never_initializes_real_sdk():
    supervisor = ControlPCSupervisor(config(), arm="both")
    assert supervisor.adapter.enable_real is False
    assert supervisor.adapter.initialized is False
    assert supervisor.status_snapshot().teleop_state == "IDLE"


def test_status_command_reports_full_runtime_gate_state():
    supervisor = ControlPCSupervisor(config(), arm="left")
    supervisor.start()
    try:
        result = supervisor.execute_command("status")
        status = json.loads(result.message)
        assert status["dry_run"] is True
        assert status["teleop_state"] == "IDLE"
        assert status["control_rights"] is True
        assert status["arm_sides"] == ["left"]
        assert status["use_wbc"] is False
        assert status["add_default_torso"] is False
        assert status["locked_groups"] == ["torso", "chassis", "head"]
        assert status["position_scale_xyz"] == [0.3, 0.3, 0.3]
    finally:
        supervisor.close()


def test_supervisor_rejects_injected_real_adapter_before_initialization():
    adapter = AstribotAdapter(enable_real=True, robot_factory=lambda **kwargs: None)

    with pytest.raises(RuntimeError, match="real/dry-run mode"):
        ControlPCSupervisor(config(), adapter=adapter)

    assert adapter.initialized is False


def test_m8_real_supervisor_is_left_relative_paused_and_does_not_initialize_until_start():
    adapter = AstribotAdapter(enable_real=True, robot_factory=lambda **kwargs: None)
    supervisor = ControlPCSupervisor(
        config(), arm="left", mapping_mode="relative", enable_real_arm=True, adapter=adapter
    )

    assert supervisor.enabled_sides == ("left",)
    assert supervisor.status_snapshot().dry_run is False
    assert supervisor.status_snapshot().teleop_state == "IDLE"
    assert adapter.initialized is False
    assert adapter.stats.send_calls == 0


def test_m8_real_supervisor_allows_dual_relative_without_initializing_until_start():
    adapter = AstribotAdapter(enable_real=True, robot_factory=lambda **kwargs: None)
    supervisor = ControlPCSupervisor(
        config(), arm="both", mapping_mode="relative", enable_real_arm=True, adapter=adapter
    )

    assert supervisor.enabled_sides == ("left", "right")
    assert supervisor.status_snapshot().teleop_state == "IDLE"
    assert adapter.initialized is False
    assert adapter.stats.send_calls == 0


def test_m8_real_supervisor_allows_explicit_dual_absolute_without_initializing():
    adapter = AstribotAdapter(enable_real=True, robot_factory=lambda **kwargs: None)
    supervisor = ControlPCSupervisor(
        config(),
        arm="both",
        mapping_mode="absolute",
        enable_real_arm=True,
        adapter=adapter,
        allow_real_absolute=True,
    )

    assert supervisor.enabled_sides == ("left", "right")
    assert supervisor.status_snapshot().mapping_mode == "absolute"
    assert adapter.initialized is False


def test_wait_until_adapter_ready_reports_control_loop_initialization_fault():
    adapter = AstribotAdapter(enable_real=True, robot_factory=lambda **kwargs: None)
    supervisor = ControlPCSupervisor(
        config(), arm="left", mapping_mode="relative", enable_real_arm=True, adapter=adapter
    )
    supervisor.start()
    with pytest.raises(RuntimeError, match="control loop exception"):
        supervisor.wait_until_adapter_ready(1.0)


@pytest.mark.parametrize("arm, mode", [("right", "relative")])
def test_m8_real_supervisor_rejects_out_of_scope_arm_or_mode(arm, mode):
    with pytest.raises(RuntimeError, match="left or both arms"):
        ControlPCSupervisor(config(), arm=arm, mapping_mode=mode, enable_real_arm=True)


@pytest.mark.parametrize("arm", ["left", "both"])
def test_m8_real_supervisor_requires_explicit_absolute_authorization(arm):
    with pytest.raises(RuntimeError, match="full-absolute authorization"):
        ControlPCSupervisor(config(), arm=arm, mapping_mode="absolute", enable_real_arm=True)


def test_m8_real_supervisor_rejects_wbc_or_locked_group_enablement():
    cfg = config()
    cfg["whole_body"]["allow_torso"] = True

    with pytest.raises(RuntimeError, match="torso, chassis, and head"):
        ControlPCSupervisor(cfg, arm="left", mapping_mode="relative", enable_real_arm=True)


def test_absolute_quality_failure_stays_paused_not_faulted():
    cfg = config()
    cfg["absolute_session_calibration"]["max_hand_position_std_m"] = 0.001
    supervisor = ControlPCSupervisor(cfg, arm="left")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("absolute-calibrate").accepted
        for seq in range(2, 8):
            frame = build_mock_frame(seq)
            frame["hands"]["left"]["positions"][0][0] = 0.02 if seq % 2 else -0.02
            supervisor.ingest_payload(frame)
            time.sleep(0.01)
        status = supervisor.status_snapshot()
        assert status.calibration_state == "INVALID"
        assert status.teleop_state == "PAUSED"
        assert status.loop_state != "FAULT"
        assert "hand position std" in status.calibration_failure_reason
    finally:
        supervisor.close()


def test_initial_absolute_mode_still_runs_post_calibration_jump_gate():
    cfg = config()
    supervisor = ControlPCSupervisor(cfg, arm="left", mapping_mode="absolute")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("absolute-calibrate").accepted
        for seq in range(2, 9):
            feed(supervisor, seq)
            time.sleep(0.01)
        status = supervisor.status_snapshot()
        assert status.calibration_state == "VALID"
        assert status.teleop_state == "ARMED"
        assert supervisor.adapter.stats.send_calls == 0
    finally:
        supervisor.close()


def test_initial_absolute_mode_rejects_post_calibration_jump():
    cfg = config()
    cfg["safety"]["mode_switch_max_position_jump_m"] = 0.001
    supervisor = ControlPCSupervisor(cfg, arm="left", mapping_mode="absolute")
    supervisor.start()
    try:
        feed(supervisor, 1)
        assert supervisor.execute_command("absolute-calibrate").accepted
        for seq in range(2, 9):
            feed(supervisor, seq)
            time.sleep(0.01)
        status = supervisor.status_snapshot()
        assert status.calibration_state == "VALID"
        assert status.teleop_state == "PAUSED"
        assert "exceeds mode-switch limits" in status.last_error
        assert supervisor.adapter.stats.send_calls == 0
    finally:
        supervisor.close()

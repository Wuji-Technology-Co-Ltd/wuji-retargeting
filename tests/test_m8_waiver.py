from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml

from stardust_wuji_quest3_pc_retargeting.hardware_audit.m8_waiver import validate_m8_waiver
from stardust_wuji_quest3_pc_retargeting.tools import run_control_pc_supervisor as cli


def arm_config():
    config = yaml.safe_load((Path(__file__).parents[1] / "configs/arm/s1_quest3_default.yaml").read_text())
    config["mapping"]["position_scale_xyz"] = [1.0, 1.0, 1.0]
    config["safety"]["max_linear_speed_mps"] = 0.20
    config["safety"]["max_input_position_jump_m"] = 0.03
    config["safety"]["mode_switch_max_position_jump_m"] = 0.01
    config["filter"]["position_alpha"] = 0.70
    return config


def write_waiver(tmp_path, report_content=b"report"):
    report = tmp_path / "report.yaml"
    report.write_bytes(report_content)
    waiver = {
        "schema": "astribot_s1_m8_operator_waiver.v1",
        "identity_assurance": "self_attested_unverified",
        "decision": "approved_with_known_evidence_gaps",
        "bound_m7_report": {"path": str(report), "sha256": hashlib.sha256(report_content).hexdigest()},
        "authorized_scope": {
            "milestone": "M8",
            "arm": "both",
            "authorized_arm_modes": ["left", "both"],
            "mapping_mode": "relative",
            "enable_orientation": False,
            "use_wbc": False,
            "add_default_torso": False,
            "locked_groups": ["torso", "chassis", "head"],
            "position_scale_xyz_max": [2.0, 2.0, 2.0],
            "max_linear_speed_mps": 1.00,
            "position_alpha_max": 1.00,
        },
        "control_rights_takeover": {
            "authorized": True,
            "method": "Astribot_high_control_rights",
            "operator_confirmation_required": "M8 FORCE TAKEOVER WEB CONTROL RIGHTS",
        },
        "high_speed_authorization": {
            "authorized": True,
            "maximum_linear_speed_mps": 1.00,
            "operator_confirmation_required": "M8 HIGH SPEED 1.0 MPS PHYSICAL ESTOP",
        },
        "orientation_authorization": {
            "authorized": True,
            "maximum_rotation_scale": 1.00,
            "maximum_angular_speed_rad_s": 3.00,
            "maximum_input_rotation_jump_rad": 0.35,
            "operator_confirmation_required": "M8 ENABLE ORIENTATION PHYSICAL ESTOP",
            "high_rate_confirmation_required": "M8 HIGH RATE ORIENTATION PHYSICAL ESTOP",
        },
        "dual_arm_authorization": {
            "authorized": True,
            "sdk_command_mode": "single_dual_arm_batch",
        },
        "tracking_reacquire_authorization": {
            "authorized": True,
            "maximum_timeout_sec": 10.0,
            "minimum_stable_frames": 3,
            "recovery_strategy": "relative_reanchor_all_enabled_arms",
            "absolute_orientation_catchup_authorized": True,
            "maximum_catchup_angular_speed_rad_s": 1.0,
            "maximum_automatic_orientation_error_rad": 1.57,
            "catchup_position_policy": "hold_then_reanchor",
        },
        "attestation": {
            "operator_accepts_residual_risk": True,
            "waiver_is_not_m7_completion_evidence": True,
            "bundled_confirmation_token": "M8_ACCEPT_ALL_AUTHORIZED_RISKS",
        },
    }
    path = tmp_path / "waiver.yaml"
    path.write_text(yaml.safe_dump(waiver), encoding="utf-8")
    return path, report


def test_valid_m8_waiver_authorizes_left_and_dual_relative_configurations(tmp_path):
    waiver, _ = write_waiver(tmp_path)

    loaded = validate_m8_waiver(waiver, arm_config(), "left", "relative")

    assert loaded["authorized_scope"]["authorized_arm_modes"] == ["left", "both"]
    validate_m8_waiver(waiver, arm_config(), "left", "relative", require_control_takeover=True)
    validate_m8_waiver(waiver, arm_config(), "both", "relative", require_control_takeover=True)


def test_m8_waiver_rejects_changed_report(tmp_path):
    waiver, report = write_waiver(tmp_path)
    report.write_text("changed", encoding="utf-8")

    with pytest.raises(RuntimeError, match="hash changed"):
        validate_m8_waiver(waiver, arm_config(), "left", "relative")


@pytest.mark.parametrize("arm, mode", [("right", "relative"), ("left", "absolute")])
def test_m8_waiver_rejects_scope_expansion(tmp_path, arm, mode):
    waiver, _ = write_waiver(tmp_path)

    with pytest.raises(RuntimeError, match="exceeds waiver scope"):
        validate_m8_waiver(waiver, arm_config(), arm, mode)


def test_real_cli_requires_flag_waiver_and_exact_physical_confirmation(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))

    missing_flag = cli.parse_args(["--arm", "left", "--mapping-mode", "relative", "--enable-real-arm"])
    with pytest.raises(RuntimeError, match="confirm-m8-real-arm"):
        cli.build_supervisor(missing_flag)

    missing_waiver = cli.parse_args(
        ["--arm", "left", "--mapping-mode", "relative", "--enable-real-arm", "--confirm-m8-real-arm"]
    )
    with pytest.raises(RuntimeError, match="requires --m8-waiver"):
        cli.build_supervisor(missing_waiver)

    args = cli.parse_args(
        [
            "--arm", "left", "--mapping-mode", "relative", "--enable-real-arm",
            "--confirm-m8-real-arm", "--m8-waiver", str(waiver),
        ]
    )
    monkeypatch.setattr("builtins.input", lambda prompt: "wrong")
    with pytest.raises(RuntimeError, match="confirmation did not match"):
        cli.build_supervisor(args)

    monkeypatch.setattr("builtins.input", lambda prompt: "M8 LEFT RELATIVE PHYSICAL ESTOP")
    supervisor = cli.build_supervisor(args)
    assert supervisor.adapter.enable_real is True
    assert supervisor.adapter.initialized is False
    assert supervisor.enabled_sides == ("left",)


def test_real_cli_high_speed_requires_flag_and_third_confirmation(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))
    base = [
        "--arm", "left", "--mapping-mode", "relative", "--enable-real-arm",
        "--confirm-m8-real-arm", "--allow-control-takeover", "--m8-waiver", str(waiver),
        "--m8-max-linear-speed-mps", "1.0",
    ]
    with pytest.raises(RuntimeError, match="confirm-m8-high-speed"):
        cli.build_supervisor(cli.parse_args(base))

    answers = iter(
        [
            "M8 LEFT RELATIVE PHYSICAL ESTOP",
            "M8 FORCE TAKEOVER WEB CONTROL RIGHTS",
            "M8 HIGH SPEED 1.0 MPS PHYSICAL ESTOP",
        ]
    )
    monkeypatch.setattr("builtins.input", lambda prompt: next(answers))
    supervisor = cli.build_supervisor(cli.parse_args(base + ["--confirm-m8-high-speed"]))
    assert supervisor.status_snapshot().max_linear_speed_mps == 1.0


def test_real_cli_orientation_requires_confirmation_and_applies_limits(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))
    base = [
        "--arm", "left", "--mapping-mode", "relative", "--enable-real-arm",
        "--confirm-m8-real-arm", "--m8-waiver", str(waiver), "--enable-m8-orientation",
    ]
    with pytest.raises(RuntimeError, match="confirm-m8-orientation"):
        cli.build_supervisor(cli.parse_args(base))
    answers = iter(["M8 LEFT RELATIVE PHYSICAL ESTOP", "M8 ENABLE ORIENTATION PHYSICAL ESTOP"])
    monkeypatch.setattr("builtins.input", lambda prompt: next(answers))
    supervisor = cli.build_supervisor(cli.parse_args(base + ["--confirm-m8-orientation"]))
    status = supervisor.status_snapshot()
    assert status.enable_orientation is True
    assert status.rotation_scale == 0.3
    assert status.max_angular_speed_rad_s == 0.3


def test_real_cli_high_rate_orientation_requires_extra_confirmation(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))
    base = [
        "--arm", "left", "--mapping-mode", "relative", "--enable-real-arm",
        "--confirm-m8-real-arm", "--m8-waiver", str(waiver),
        "--enable-m8-orientation", "--confirm-m8-orientation",
        "--m8-rotation-scale", "0.7", "--m8-max-angular-speed-rad-s", "1.0",
    ]
    with pytest.raises(RuntimeError, match="confirm-m8-high-rate-orientation"):
        cli.build_supervisor(cli.parse_args(base))
    answers = iter(
        [
            "M8 LEFT RELATIVE PHYSICAL ESTOP",
            "M8 ENABLE ORIENTATION PHYSICAL ESTOP",
            "M8 HIGH RATE ORIENTATION PHYSICAL ESTOP",
        ]
    )
    monkeypatch.setattr("builtins.input", lambda prompt: next(answers))
    supervisor = cli.build_supervisor(cli.parse_args(base + ["--confirm-m8-high-rate-orientation"]))
    status = supervisor.status_snapshot()
    assert status.rotation_scale == 0.7
    assert status.max_angular_speed_rad_s == 1.0


def test_real_cli_rejects_incorrect_bundled_confirmation_before_loading_config(monkeypatch):
    monkeypatch.setattr(
        cli,
        "load_arm_config",
        lambda path: pytest.fail("configuration must not load for an invalid bundled token"),
    )
    args = cli.parse_args(["--accept-m8-risk-bundle", "wrong"])

    with pytest.raises(RuntimeError, match="bundled risk confirmation token did not match"):
        cli.build_supervisor(args)


def test_real_cli_bundled_confirmation_skips_all_interactive_prompts(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))
    monkeypatch.setattr(
        "builtins.input",
        lambda prompt: pytest.fail(f"bundled confirmation unexpectedly prompted: {prompt}"),
    )
    args = cli.parse_args(
        [
            "--arm", "left",
            "--mapping-mode", "relative",
            "--enable-real-arm",
            "--allow-control-takeover",
            "--m8-max-linear-speed-mps", "1.0",
            "--m8-position-alpha", "0.9",
            "--enable-m8-orientation",
            "--m8-rotation-scale", "1.0",
            "--m8-max-angular-speed-rad-s", "3.0",
            "--m8-waiver", str(waiver),
            "--accept-m8-risk-bundle", "M8_ACCEPT_ALL_AUTHORIZED_RISKS",
        ]
    )

    supervisor = cli.build_supervisor(args)
    status = supervisor.status_snapshot()
    assert supervisor.adapter.enable_real is True
    assert status.max_linear_speed_mps == 1.0
    assert status.position_alpha == 0.9
    assert status.enable_orientation is True
    assert status.rotation_scale == 1.0
    assert status.max_angular_speed_rad_s == 3.0


def test_real_cli_dual_arm_applies_waiver_bounded_position_scale(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))
    monkeypatch.setattr(
        "builtins.input",
        lambda prompt: pytest.fail(f"bundled confirmation unexpectedly prompted: {prompt}"),
    )
    args = cli.parse_args(
        [
            "--arm", "both",
            "--mapping-mode", "relative",
            "--enable-real-arm",
            "--allow-control-takeover",
            "--m8-position-scale", "1.5",
            "--m8-hand-reacquire-timeout-sec", "5.0",
            "--enable-m8-orientation",
            "--m8-rotation-scale", "1.0",
            "--enable-m8-absolute-orientation-reacquire",
            "--m8-orientation-reacquire-speed-rad-s", "0.5",
            "--m8-orientation-reacquire-max-error-rad", "1.57",
            "--m8-waiver", str(waiver),
            "--accept-m8-risk-bundle", "M8_ACCEPT_ALL_AUTHORIZED_RISKS",
        ]
    )

    supervisor = cli.build_supervisor(args)
    status = supervisor.status_snapshot()
    assert supervisor.enabled_sides == ("left", "right")
    assert status.position_scale_xyz == (1.5, 1.5, 1.5)
    assert status.hand_reacquire_timeout_sec == 5.0
    assert status.absolute_orientation_reacquire is True
    assert status.orientation_reacquire_speed_rad_s == 0.5


def test_real_cli_hand_reacquire_timeout_is_bounded_by_waiver(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))
    args = cli.parse_args(
        [
            "--arm", "both",
            "--mapping-mode", "relative",
            "--enable-real-arm",
            "--m8-hand-reacquire-timeout-sec", "10.1",
            "--m8-waiver", str(waiver),
            "--accept-m8-risk-bundle", "M8_ACCEPT_ALL_AUTHORIZED_RISKS",
        ]
    )

    with pytest.raises(RuntimeError, match="reacquire policy"):
        cli.build_supervisor(args)


def test_real_cli_orientation_rate_is_bounded_by_waiver_not_a_cli_constant(monkeypatch, tmp_path):
    waiver, _ = write_waiver(tmp_path)
    service = {"arms": {"sdk_root": "/vendor/sdk"}}
    monkeypatch.setattr(cli, "load_arm_config", lambda path: (service, arm_config()))
    args = cli.parse_args(
        [
            "--arm", "left",
            "--mapping-mode", "relative",
            "--enable-real-arm",
            "--enable-m8-orientation",
            "--m8-rotation-scale", "1.0",
            "--m8-max-angular-speed-rad-s", "3.1",
            "--m8-waiver", str(waiver),
            "--accept-m8-risk-bundle", "M8_ACCEPT_ALL_AUTHORIZED_RISKS",
        ]
    )

    with pytest.raises(RuntimeError, match="waiver does not authorize"):
        cli.build_supervisor(args)

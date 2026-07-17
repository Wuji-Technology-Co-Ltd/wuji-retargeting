from __future__ import annotations

import hashlib
from math import isfinite
from pathlib import Path
from typing import Any

import yaml

from stardust_wuji_quest3_pc_retargeting.runtime.config import repo_root


WAIVER_SCHEMA = "astribot_s1_m8_operator_waiver.v1"


def validate_m8_waiver(
    path: str | Path,
    arm_config: dict[str, Any],
    arm: str,
    mapping_mode: str,
    require_control_takeover: bool = False,
) -> dict[str, Any]:
    waiver_path = Path(path).expanduser().resolve()
    waiver = yaml.safe_load(waiver_path.read_text(encoding="utf-8")) or {}
    if waiver.get("schema") != WAIVER_SCHEMA:
        raise RuntimeError("M8 waiver schema is invalid")
    if waiver.get("identity_assurance") != "self_attested_unverified":
        raise RuntimeError("M8 waiver identity assurance is missing")
    if waiver.get("decision") != "approved_with_known_evidence_gaps":
        raise RuntimeError("M8 waiver is not approved")
    attestation = waiver.get("attestation", {})
    if attestation.get("operator_accepts_residual_risk") is not True:
        raise RuntimeError("M8 waiver does not accept residual risk")
    if attestation.get("waiver_is_not_m7_completion_evidence") is not True:
        raise RuntimeError("M8 waiver must preserve the M7 evidence boundary")

    bound = waiver.get("bound_m7_report", {})
    report_path = Path(str(bound.get("path", ""))).expanduser()
    if not report_path.is_absolute():
        report_path = repo_root() / report_path
    actual_hash = hashlib.sha256(report_path.read_bytes()).hexdigest()
    if actual_hash != bound.get("sha256"):
        raise RuntimeError("M8 waiver is stale because the bound M7 report hash changed")

    scope = waiver.get("authorized_scope", {})
    mapping = arm_config.get("mapping", {})
    safety = arm_config.get("safety", {})
    authorized_arm_modes = scope.get("authorized_arm_modes")
    if not isinstance(authorized_arm_modes, list):
        authorized_arm_modes = [scope.get("arm")]
    authorized_mapping_modes = scope.get("authorized_mapping_modes")
    if not isinstance(authorized_mapping_modes, list):
        authorized_mapping_modes = [scope.get("mapping_mode")]
    requested_position_scale = mapping.get("position_scale_xyz")
    maximum_position_scale = scope.get("position_scale_xyz_max", scope.get("position_scale_xyz"))
    position_scale_valid = (
        isinstance(requested_position_scale, list)
        and isinstance(maximum_position_scale, list)
        and len(requested_position_scale) == len(maximum_position_scale) == 3
        and all(isfinite(float(value)) and float(value) > 0.0 for value in requested_position_scale)
        and all(isfinite(float(value)) and float(value) > 0.0 for value in maximum_position_scale)
        and all(
            float(requested) <= float(maximum)
            for requested, maximum in zip(requested_position_scale, maximum_position_scale)
        )
    )
    checks = {
        "milestone": scope.get("milestone") == "M8",
        "arm": arm in authorized_arm_modes,
        "mapping_mode": mapping_mode in authorized_mapping_modes,
        "use_wbc": arm_config.get("use_wbc") is scope.get("use_wbc") is False,
        "add_default_torso": arm_config.get("add_default_torso") is scope.get("add_default_torso") is False,
        "locked_groups": scope.get("locked_groups") == ["torso", "chassis", "head"],
        "position_scale_xyz": position_scale_valid,
        "max_linear_speed_mps": 0.0 < float(safety.get("max_linear_speed_mps", -1.0))
        <= float(scope.get("max_linear_speed_mps", -1.0))
        <= 2.00,
        "position_alpha": float(arm_config.get("filter", {}).get("position_alpha", -1.0))
        <= float(scope.get("position_alpha_max", -1.0))
        == 1.00,
    }
    failed = [name for name, valid in checks.items() if not valid]
    if failed:
        raise RuntimeError("M8 request exceeds waiver scope: " + ", ".join(failed))
    if arm == "both":
        dual_arm = waiver.get("dual_arm_authorization", {})
        if dual_arm.get("authorized") is not True or dual_arm.get("sdk_command_mode") != "single_dual_arm_batch":
            raise RuntimeError("M8 waiver does not authorize dual-arm control")
    hand_reacquire_timeout = float(safety.get("hand_reacquire_timeout_sec", 0.0))
    if hand_reacquire_timeout > 0.0:
        tracking_reacquire = waiver.get("tracking_reacquire_authorization", {})
        recovery_strategies = tracking_reacquire.get("recovery_strategies")
        if not isinstance(recovery_strategies, list):
            recovery_strategies = [tracking_reacquire.get("recovery_strategy")]
        expected_recovery_strategy = (
            "fixed_anchor_pose_catchup_all_enabled_arms"
            if bool(safety.get("fixed_anchor_pose_reacquire", False))
            else (
                "absolute_pose_catchup_all_enabled_arms"
                if mapping_mode == "absolute"
                else "relative_reanchor_all_enabled_arms"
            )
        )
        if (
            tracking_reacquire.get("authorized") is not True
            or hand_reacquire_timeout > float(tracking_reacquire.get("maximum_timeout_sec", 0.0))
            or int(safety.get("hand_reacquire_stable_frames", 0))
            < int(tracking_reacquire.get("minimum_stable_frames", 0))
            or expected_recovery_strategy not in recovery_strategies
        ):
            raise RuntimeError("M8 waiver does not authorize the requested hand-tracking reacquire policy")
        if bool(safety.get("absolute_orientation_reacquire", False)):
            if (
                tracking_reacquire.get("absolute_orientation_catchup_authorized") is not True
                or float(safety.get("orientation_reacquire_speed_rad_s", 0.0))
                > float(tracking_reacquire.get("maximum_catchup_angular_speed_rad_s", 0.0))
                or float(safety.get("orientation_reacquire_max_error_rad", 0.0))
                > float(tracking_reacquire.get("maximum_automatic_orientation_error_rad", 0.0))
                or tracking_reacquire.get("catchup_position_policy") != "hold_then_reanchor"
            ):
                raise RuntimeError("M8 waiver does not authorize absolute-orientation catch-up")
        if bool(safety.get("fixed_anchor_mode", False)):
            fixed_anchor = waiver.get("fixed_anchor_authorization", {})
            if (
                fixed_anchor.get("authorized") is not True
                or fixed_anchor.get("orientation_anchor_policy") != "preserve_until_explicit_recalibration"
                or float(safety.get("absolute_reacquire_linear_speed_mps", 0.0))
                > float(fixed_anchor.get("maximum_reacquire_linear_speed_mps", 0.0))
                or float(safety.get("absolute_reacquire_max_position_error_m", 0.0))
                > float(fixed_anchor.get("maximum_automatic_position_error_m", 0.0))
                or float(safety.get("orientation_reacquire_speed_rad_s", 0.0))
                > float(fixed_anchor.get("maximum_reacquire_angular_speed_rad_s", 0.0))
                or float(safety.get("orientation_reacquire_max_error_rad", 0.0))
                > float(fixed_anchor.get("maximum_automatic_orientation_error_rad", 0.0))
            ):
                raise RuntimeError("M8 waiver does not authorize fixed-anchor control")
    if mapping_mode == "absolute":
        full_absolute = waiver.get("full_absolute_authorization", {})
        if (
            full_absolute.get("authorized") is not True
            or full_absolute.get("requires_session_calibration") is not True
            or bool(safety.get("absolute_pose_reacquire", False)) is not True
            or float(safety.get("absolute_reacquire_linear_speed_mps", 0.0))
            > float(full_absolute.get("maximum_reacquire_linear_speed_mps", 0.0))
            or float(safety.get("absolute_reacquire_max_position_error_m", 0.0))
            > float(full_absolute.get("maximum_automatic_position_error_m", 0.0))
            or float(safety.get("orientation_reacquire_speed_rad_s", 0.0))
            > float(full_absolute.get("maximum_reacquire_angular_speed_rad_s", 0.0))
            or float(safety.get("orientation_reacquire_max_error_rad", 0.0))
            > float(full_absolute.get("maximum_automatic_orientation_error_rad", 0.0))
        ):
            raise RuntimeError("M8 waiver does not authorize full absolute control")
    if require_control_takeover:
        takeover = waiver.get("control_rights_takeover", {})
        if (
            takeover.get("authorized") is not True
            or takeover.get("method") != "Astribot_high_control_rights"
            or takeover.get("operator_confirmation_required") != "M8 FORCE TAKEOVER WEB CONTROL RIGHTS"
        ):
            raise RuntimeError("M8 waiver does not authorize high-control-rights takeover")
    requested_linear_speed = float(safety.get("max_linear_speed_mps", 0.0))
    if requested_linear_speed > 1.00:
        very_high_speed = waiver.get("very_high_speed_authorization", {})
        if (
            very_high_speed.get("authorized") is not True
            or requested_linear_speed
            > float(very_high_speed.get("maximum_linear_speed_mps", 0.0))
            or very_high_speed.get("operator_confirmation_required")
            != "M8 VERY HIGH SPEED 2.0 MPS PHYSICAL ESTOP"
        ):
            raise RuntimeError("M8 waiver does not authorize the requested very-high-speed mode")
    elif requested_linear_speed > 0.20:
        high_speed = waiver.get("high_speed_authorization", {})
        if (
            high_speed.get("authorized") is not True
            or float(high_speed.get("maximum_linear_speed_mps", 0.0)) != 1.00
            or high_speed.get("operator_confirmation_required") != "M8 HIGH SPEED 1.0 MPS PHYSICAL ESTOP"
        ):
            raise RuntimeError("M8 waiver does not authorize the requested high-speed mode")
    position_lead_sec = float(arm_config.get("filter", {}).get("position_lead_sec", 0.0))
    max_position_lead_m = float(arm_config.get("filter", {}).get("max_position_lead_m", 0.0))
    if position_lead_sec > 0.0:
        lead = waiver.get("position_lead_authorization", {})
        if (
            lead.get("authorized") is not True
            or position_lead_sec > float(lead.get("maximum_lead_sec", 0.0))
            or max_position_lead_m > float(lead.get("maximum_lead_distance_m", 0.0))
            or lead.get("operator_confirmation_required") != "M8 POSITION LEAD PHYSICAL ESTOP"
        ):
            raise RuntimeError("M8 waiver does not authorize the requested position lead")
    if bool(safety.get("init_recovery_enabled", False)):
        init_recovery = waiver.get("init_recovery_authorization", {})
        requested_recovery = arm_config.get("init_recovery", {})
        requested_arms = requested_recovery.get("arms", {})
        authorized_arms = init_recovery.get("arm_joint_targets", {})
        source_path = Path(str(requested_recovery.get("source", ""))).expanduser()
        source_hash = ""
        if source_path.is_file():
            source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
        if (
            init_recovery.get("authorized") is not True
            or init_recovery.get("body_parts") != ["astribot_arm_left", "astribot_arm_right"]
            or requested_arms != authorized_arms
            or float(requested_recovery.get("duration_sec", 0.0))
            > float(init_recovery.get("maximum_duration_sec", 0.0))
            or float(requested_recovery.get("joint_tolerance_rad", 1.0))
            > float(init_recovery.get("maximum_joint_tolerance_rad", 0.0))
            or source_hash != init_recovery.get("source_sha256")
            or init_recovery.get("operator_confirmation_required")
            != "M8 INIT JOINT RECOVERY PHYSICAL ESTOP"
        ):
            raise RuntimeError("M8 waiver does not authorize the requested init-joint recovery")
    if bool(mapping.get("enable_orientation", False)):
        orientation = waiver.get("orientation_authorization", {})
        safety = arm_config.get("safety", {})
        if (
            orientation.get("authorized") is not True
            or float(mapping.get("rotation_scale", 0.0)) > float(orientation.get("maximum_rotation_scale", 0.0))
            or float(safety.get("max_angular_speed_rad_s", 0.0))
            > float(orientation.get("maximum_angular_speed_rad_s", 0.0))
            or float(safety.get("max_input_rotation_jump_rad", 0.0))
            > float(orientation.get("maximum_input_rotation_jump_rad", 0.0))
            or orientation.get("operator_confirmation_required") != "M8 ENABLE ORIENTATION PHYSICAL ESTOP"
        ):
            raise RuntimeError("M8 waiver does not authorize the requested orientation mode")
        if (
            float(mapping.get("rotation_scale", 0.0)) > 0.30
            or float(safety.get("max_angular_speed_rad_s", 0.0)) > 0.30
        ) and orientation.get("high_rate_confirmation_required") != "M8 HIGH RATE ORIENTATION PHYSICAL ESTOP":
            raise RuntimeError("M8 waiver does not authorize high-rate orientation")
    return waiver

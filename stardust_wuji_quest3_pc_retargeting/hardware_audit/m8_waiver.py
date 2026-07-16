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
        "mapping_mode": mapping_mode == scope.get("mapping_mode") == "relative",
        "use_wbc": arm_config.get("use_wbc") is scope.get("use_wbc") is False,
        "add_default_torso": arm_config.get("add_default_torso") is scope.get("add_default_torso") is False,
        "locked_groups": scope.get("locked_groups") == ["torso", "chassis", "head"],
        "position_scale_xyz": position_scale_valid,
        "max_linear_speed_mps": 0.0 < float(safety.get("max_linear_speed_mps", -1.0))
        <= float(scope.get("max_linear_speed_mps", -1.0))
        == 1.00,
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
        if (
            tracking_reacquire.get("authorized") is not True
            or hand_reacquire_timeout > float(tracking_reacquire.get("maximum_timeout_sec", 0.0))
            or int(safety.get("hand_reacquire_stable_frames", 0))
            < int(tracking_reacquire.get("minimum_stable_frames", 0))
            or tracking_reacquire.get("recovery_strategy") != "relative_reanchor_all_enabled_arms"
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
    if require_control_takeover:
        takeover = waiver.get("control_rights_takeover", {})
        if (
            takeover.get("authorized") is not True
            or takeover.get("method") != "Astribot_high_control_rights"
            or takeover.get("operator_confirmation_required") != "M8 FORCE TAKEOVER WEB CONTROL RIGHTS"
        ):
            raise RuntimeError("M8 waiver does not authorize high-control-rights takeover")
    if float(safety.get("max_linear_speed_mps", 0.0)) > 0.20:
        high_speed = waiver.get("high_speed_authorization", {})
        if (
            high_speed.get("authorized") is not True
            or float(high_speed.get("maximum_linear_speed_mps", 0.0)) != 1.00
            or high_speed.get("operator_confirmation_required") != "M8 HIGH SPEED 1.0 MPS PHYSICAL ESTOP"
        ):
            raise RuntimeError("M8 waiver does not authorize the requested high-speed mode")
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

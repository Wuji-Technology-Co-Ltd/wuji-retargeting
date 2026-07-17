import numpy as np
import pytest

from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmTarget
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_angle_xyzw, quat_from_yaw_y_up
from stardust_wuji_quest3_pc_retargeting.safety.arm_safety_filter import ArmSafetyFilter, ArmSafetyState


def target(position, orientation=(0, 0, 0, 1)):
    return ArmTarget(list(position), list(orientation))


@pytest.mark.parametrize("dt", [0.005, 0.01, 0.02])
def test_vector_linear_and_angular_speed_limits_use_actual_dt(dt):
    filt = ArmSafetyFilter(
        max_linear_speed_mps=0.2,
        max_angular_speed_rad_s=0.5,
        max_input_position_jump_m=10,
        max_input_rotation_jump_rad=10,
    )
    first = filt.filter(target([0, 0, 0]), valid=True, running=True, dt_sec=dt)
    second = filt.filter(target([1, 1, 0], quat_from_yaw_y_up(1.0)), valid=True, running=True, dt_sec=dt)

    assert np.linalg.norm(np.asarray(second.target.position) - first.target.position) == pytest.approx(0.2 * dt)
    assert quat_angle_xyzw(first.target.orientation_xyzw, second.target.orientation_xyzw) == pytest.approx(0.5 * dt)


def test_workspace_clamps_and_invalid_values_never_become_enabled():
    filt = ArmSafetyFilter(xyz_min=[-1, -1, -1], xyz_max=[1, 1, 1])
    clipped = filt.filter(target([2, 0, 0]), valid=True, running=True, dt_sec=0.01)
    invalid = filt.filter(target([0, 0, 0], [0, 0, 0, 0]), valid=True, running=True, dt_sec=0.01)

    assert clipped.workspace_clipped is True
    np.testing.assert_allclose(clipped.target.position, [1, 0, 0])
    assert invalid.enabled is False
    assert invalid.state is ArmSafetyState.FAULT


def test_tracking_lost_holds_without_advancing_old_velocity():
    filt = ArmSafetyFilter(max_input_position_jump_m=10)
    filt.filter(target([0, 0, 0]), valid=True, running=True, dt_sec=0.01)
    moving = filt.filter(target([1, 0, 0]), valid=True, running=True, dt_sec=0.01)
    lost = filt.filter(target([2, 0, 0]), valid=False, running=True, dt_sec=0.01)

    assert lost.enabled is False
    assert lost.state is ArmSafetyState.HOLD
    np.testing.assert_allclose(lost.target.position, moving.target.position)


def test_discontinuity_and_excessive_dt_fail_closed():
    filt = ArmSafetyFilter(max_input_position_jump_m=0.1, maximum_dt_sec=0.05)
    filt.filter(target([0, 0, 0]), valid=True, running=True, dt_sec=0.01)

    jump = filt.filter(target([1, 0, 0]), valid=True, running=True, dt_sec=0.01)
    timing = filt.filter(target([0.01, 0, 0]), valid=True, running=True, dt_sec=0.2)

    assert jump.enabled is False and "jump" in jump.reason
    assert timing.state is ArmSafetyState.FAULT and "control dt" in timing.reason


def test_bounded_position_lead_predicts_velocity_without_changing_alpha_semantics():
    filt = ArmSafetyFilter(
        max_linear_speed_mps=10.0,
        max_input_position_jump_m=10.0,
        position_alpha=1.0,
        position_lead_sec=0.05,
        max_position_lead_m=0.05,
    )
    filt.filter(target([0, 0, 0]), valid=True, running=True, dt_sec=0.01)
    predicted = filt.filter(target([0.01, 0, 0]), valid=True, running=True, dt_sec=0.01)

    assert predicted.target.position[0] == pytest.approx(0.06)


def test_alpha_above_one_remains_rejected_as_oscillatory_gain():
    with pytest.raises(ValueError, match=r"\(0, 1\]"):
        ArmSafetyFilter(position_alpha=1.01)

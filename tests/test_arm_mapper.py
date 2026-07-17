import numpy as np
import pytest

from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmMapper, ArmTarget
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_from_yaw_y_up, quat_angle_xyzw


IDENTITY = [0.0, 0.0, 0.0, 1.0]


def target(position, orientation=IDENTITY):
    return ArmTarget(list(position), list(orientation))


def test_engage_zero_motion_is_exact_robot_pose_and_recenter_is_independent():
    mapper = ArmMapper(position_scale_xyz=[2.0, 3.0, 4.0])
    left_robot = target([0.5, 0.1, 0.8])
    right_robot = target([0.5, -0.1, 0.8])
    mapper.engage("left", target([1, 2, 3]), left_robot)
    mapper.engage("right", target([-1, 2, 3]), right_robot)

    np.testing.assert_allclose(mapper.map_hand("left", target([1, 2, 3])).as_pose_list(), left_robot.as_pose_list())
    mapper.recenter("left", target([2, 2, 3]), target([0.6, 0.1, 0.8]))

    np.testing.assert_allclose(mapper.map_hand("left", target([2, 2, 3])).position, [0.6, 0.1, 0.8])
    np.testing.assert_allclose(mapper.map_hand("right", target([-1, 2, 3])).position, right_robot.position)


def test_relative_axis_transform_and_per_axis_scale():
    rotation = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
    mapper = ArmMapper(position_scale_xyz=[0.5, 0.25, 2.0], robot_from_vr_axes=rotation)
    mapper.engage("left", target([0, 0, 0]), target([1, 2, 3]))

    mapped = mapper.map_hand("left", target([1, 2, 3]))

    np.testing.assert_allclose(mapped.position, [-0.5, 1.75, 7.0])


def test_relative_operator_yaw_reorients_hand_deltas_without_moving_anchor():
    mapper = ArmMapper(
        position_scale_xyz=[1.0, 1.0, 1.0],
        robot_from_vr_axes=np.eye(3),
    )
    mapper.set_relative_operator_yaw(np.pi / 2.0)
    mapper.engage("left", target([0, 0, 0]), target([0.4, 0.2, 0.8]))

    neutral = mapper.map_hand("left", target([0, 0, 0]))
    moved = mapper.map_hand("left", target([0.1, 0, 0]))

    np.testing.assert_allclose(neutral.position, [0.4, 0.2, 0.8], atol=1e-9)
    np.testing.assert_allclose(moved.position, [0.4, 0.2, 0.9], atol=1e-9)
    assert mapper.relative_operator_yaw_rad == pytest.approx(np.pi / 2.0)


def test_relative_rotation_scale_is_applied_and_q_sign_is_continuous():
    mapper = ArmMapper(rotation_scale=0.5, enable_orientation=True)
    mapper.engage("left", target([0, 0, 0]), target([0, 0, 0]))
    ninety = quat_from_yaw_y_up(np.pi / 2)

    first = mapper.map_hand("left", target([0, 0, 0], ninety))
    second = mapper.map_hand("left", target([0, 0, 0], -ninety))

    assert quat_angle_xyzw(first.orientation_xyzw, IDENTITY) == pytest.approx(np.pi / 4)
    assert np.dot(first.orientation_xyzw, second.orientation_xyzw) > 0.999999


def test_position_only_reanchor_preserves_absolute_orientation_alignment():
    mapper = ArmMapper(rotation_scale=1.0, enable_orientation=True)
    mapper.engage("left", target([0, 0, 0]), target([0.4, 0.3, 1.0]))
    rotated = target([0.25, 0, 0], quat_from_yaw_y_up(np.pi / 3))

    mapper.reanchor_position_only("left", rotated, target([0.45, 0.3, 1.0]))
    mapped = mapper.map_hand("left", rotated)

    np.testing.assert_allclose(mapped.position, [0.45, 0.3, 1.0])
    assert quat_angle_xyzw(mapped.orientation_xyzw, IDENTITY) == pytest.approx(np.pi / 3)


def test_axis_transform_must_be_proper_rotation():
    with pytest.raises(ValueError, match="determinant"):
        ArmMapper(robot_from_vr_axes=np.diag([-1, 1, 1]))

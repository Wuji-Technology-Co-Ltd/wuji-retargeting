import numpy as np
import pytest

from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import (
    AbsoluteCalibrationConfig,
    AbsoluteCalibrationSample,
    PoseSample,
    build_absolute_calibration,
)
from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmMapper, ArmTarget, MappingMode
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_from_yaw_y_up


def pose(position, orientation=(0, 0, 0, 1)):
    return PoseSample(tuple(position), tuple(orientation))


def calibration():
    samples = []
    for index in range(4):
        robot = {"left": pose((0.4, 0.3, 1.0))}
        samples.append(
            AbsoluteCalibrationSample(
                index,
                "session-a",
                "local-floor",
                2,
                pose((1.0, 1.6, 2.0), quat_from_yaw_y_up(0.4)),
                {"left": pose((1.2, 1.2, 1.7), quat_from_yaw_y_up(0.7))},
                robot,
                robot,
            )
        )
    cfg = AbsoluteCalibrationConfig(countdown_sec=0, sample_duration_sec=0, minimum_valid_samples=4)
    return build_absolute_calibration(samples, ["left"], np.eye(3), cfg)


def test_absolute_first_target_equals_robot_anchor_and_live_hmd_is_irrelevant():
    result = calibration()
    mapper = ArmMapper(mapping_mode=MappingMode.ABSOLUTE, position_scale_xyz=[0.5, 0.5, 0.5])
    mapper.set_absolute_calibration(result)
    hand = ArmTarget([1.2, 1.2, 1.7], quat_from_yaw_y_up(0.7).tolist())

    first = mapper.map_hand(
        "left", hand, xr_session_id="session-a", reference_space="local-floor", reference_space_revision=2
    )

    np.testing.assert_allclose(first.as_pose_list(), result.sides["left"].robot_anchor.position + result.sides["left"].robot_anchor.orientation_xyzw)
    # No live HMD pose is accepted by the API, so changing yaw/pitch/roll cannot affect the same hand pose.
    second = mapper.map_hand(
        "left", hand, xr_session_id="session-a", reference_space="local-floor", reference_space_revision=2
    )
    np.testing.assert_allclose(second.as_pose_list(), first.as_pose_list())


@pytest.mark.parametrize(
    "session, reference, revision, message",
    [
        ("other", "local-floor", 2, "session changed"),
        ("session-a", "local", 2, "reference space changed"),
        ("session-a", "local-floor", 3, "revision changed"),
    ],
)
def test_absolute_context_change_immediately_invalidates(session, reference, revision, message):
    mapper = ArmMapper(mapping_mode="absolute")
    result = calibration()
    mapper.set_absolute_calibration(result)

    with pytest.raises(RuntimeError, match=message):
        mapper.map_hand(
            "left",
            ArmTarget([1.2, 1.2, 1.7], [0, 0, 0, 1]),
            xr_session_id=session,
            reference_space=reference,
            reference_space_revision=revision,
        )

    assert result.valid is False


def test_absolute_recenter_invalidates_calibration_and_requires_new_samples():
    mapper = ArmMapper(mapping_mode="absolute")
    result = calibration()
    mapper.set_absolute_calibration(result)

    with pytest.raises(RuntimeError, match="new session calibration"):
        mapper.recenter("left", ArmTarget([0, 0, 0], [0, 0, 0, 1]), ArmTarget([0, 0, 0], [0, 0, 0, 1]))

    assert result.valid is False

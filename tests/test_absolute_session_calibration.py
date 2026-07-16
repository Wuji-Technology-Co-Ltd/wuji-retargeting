import numpy as np
import pytest

from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import (
    AbsoluteCalibrationConfig,
    AbsoluteCalibrationSample,
    AbsoluteSessionCalibrator,
    CalibrationError,
    CalibrationState,
    PoseSample,
    build_absolute_calibration,
)
from stardust_wuji_quest3_pc_retargeting.conversion.pose_math import quat_from_yaw_y_up


def pose(position, orientation=(0, 0, 0, 1)):
    return PoseSample(tuple(position), tuple(orientation))


def sample(index=0, session="session-a", reference="local-floor", revision=0, hand_offset=0.0, head_offset=0.0):
    robot = {"left": pose((0.4, 0.3, 1.0)), "right": pose((0.4, -0.3, 1.0))}
    return AbsoluteCalibrationSample(
        receive_time_ns=index * 10_000_000,
        xr_session_id=session,
        reference_space=reference,
        reference_space_revision=revision,
        hmd=pose((head_offset, 1.6, 0), quat_from_yaw_y_up(0.2)),
        hands={"left": pose((0.2 + hand_offset, 1.2, -0.3)), "right": pose((-0.2, 1.2, -0.3))},
        robot_desired=robot,
        robot_current=robot,
    )


def config(**overrides):
    values = dict(countdown_sec=0, sample_duration_sec=0.1, minimum_valid_samples=4)
    values.update(overrides)
    return AbsoluteCalibrationConfig(**values)


def test_build_calibration_freezes_floor_origin_yaw_and_session_binding():
    result = build_absolute_calibration([sample(i) for i in range(4)], ["left", "right"], np.eye(3), config())

    assert result.valid is True
    assert result.xr_session_id == "session-a"
    assert result.reference_space_revision == 0
    assert result.operator_in_vr.position[1] == 0.0
    assert set(result.sides) == {"left", "right"}
    assert result.quality["sample_count"] == 4


def test_quality_reports_specific_side_and_threshold():
    samples = [sample(i, hand_offset=0.03 if i % 2 else -0.03) for i in range(4)]

    with pytest.raises(CalibrationError, match="left hand position std"):
        build_absolute_calibration(samples, ["left"], np.eye(3), config(max_hand_position_std_m=0.005))


@pytest.mark.parametrize(
    "samples, message",
    [
        ([sample(0), sample(1)], "sample count"),
        ([sample(0), sample(1), sample(2, session="other"), sample(3)], "session/reference space changed"),
        ([sample(0, revision=None), sample(1, revision=None), sample(2, revision=None), sample(3, revision=None)], "revision is required"),
    ],
)
def test_missing_or_inconsistent_context_fails_closed(samples, message):
    with pytest.raises(CalibrationError, match=message):
        build_absolute_calibration(samples, ["left"], np.eye(3), config())


def test_calibrator_countdown_cancel_and_finish_lifecycle():
    calibrator = AbsoluteSessionCalibrator(config(countdown_sec=0.02, sample_duration_sec=0.02, minimum_valid_samples=3))
    calibrator.start(0, ["left"], np.eye(3))
    assert calibrator.state is CalibrationState.COUNTDOWN
    calibrator.add_sample(sample(1))
    assert calibrator.state is CalibrationState.COUNTDOWN
    for index in range(2, 5):
        calibrator.add_sample(sample(index))
    assert calibrator.state is CalibrationState.VALID

    calibrator.invalidate("reference space reset")
    assert calibrator.state is CalibrationState.INVALID
    assert calibrator.result is None
    assert calibrator.failure_reason == "reference space reset"


def test_calibrator_invalidates_immediately_on_mid_sample_revision_change():
    calibrator = AbsoluteSessionCalibrator(config(countdown_sec=0, sample_duration_sec=10))
    calibrator.start(0, ["left"], np.eye(3))
    calibrator.add_sample(sample(0, revision=0))

    state = calibrator.add_sample(sample(1, revision=1))

    assert state is CalibrationState.INVALID
    assert "session/reference space changed" in calibrator.failure_reason


def test_calibrator_ignores_frame_received_before_start():
    calibrator = AbsoluteSessionCalibrator(config(countdown_sec=0, sample_duration_sec=1))
    calibrator.start(100_000_000, ["left"], np.eye(3))

    state = calibrator.add_sample(sample(1))

    assert state is CalibrationState.SAMPLING
    assert calibrator.sample_count == 0

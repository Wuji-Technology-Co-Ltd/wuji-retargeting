import numpy as np

from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import AbsoluteCalibrationResult, PoseSample, SideCalibration
from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmMapper, ArmTarget, MappingMode
from stardust_wuji_quest3_pc_retargeting.runtime.arm_control_loop import ArmFrameProcessor
from stardust_wuji_quest3_pc_retargeting.safety.arm_safety_filter import ArmSafetyFilter


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def calibration(anchor=(0.4, 0.3, 1.0)):
    return AbsoluteCalibrationResult(
        "session", "local-floor", 0, 0, 1, ("left",),
        PoseSample((0, 0, 0), IDENTITY),
        {"left": SideCalibration(PoseSample((0, 0, 0), IDENTITY), PoseSample(anchor, IDENTITY), IDENTITY)},
        {},
    )


def test_running_rejects_switch_and_paused_switch_sends_nothing():
    mapper = ArmMapper()
    mapper.set_absolute_calibration(calibration())
    hand = {"left": ArmTarget([0, 0, 0], list(IDENTITY))}
    desired = {"left": ArmTarget([0.4, 0.3, 1.0], list(IDENTITY))}

    rejected = mapper.switch_mode("absolute", "RUNNING", hand, desired, ("session", "local-floor", 0))
    accepted = mapper.switch_mode("absolute", "PAUSED", hand, desired, ("session", "local-floor", 0))

    assert rejected.accepted is False
    assert accepted.accepted is True
    assert mapper.mode is MappingMode.ABSOLUTE
    assert accepted.candidates is not None  # candidates are computed only; mapper owns no adapter.


def test_mode_switch_candidate_jump_gate_fails_closed():
    mapper = ArmMapper()
    mapper.set_absolute_calibration(calibration(anchor=(1.0, 1.0, 1.0)))

    result = mapper.switch_mode(
        "absolute", "PAUSED",
        {"left": ArmTarget([0, 0, 0], list(IDENTITY))},
        {"left": ArmTarget([0, 0, 0], list(IDENTITY))},
        ("session", "local-floor", 0),
        max_position_jump_m=0.05,
    )

    assert result.accepted is False
    assert "exceeds mode-switch limits" in result.reason
    assert mapper.mode is MappingMode.RELATIVE


def test_mode_switch_rejects_candidate_outside_configured_workspace():
    mapper = ArmMapper()
    mapper.engage("left", ArmTarget([0, 0, 0], list(IDENTITY)), ArmTarget([0.1, 0.1, 0.1], list(IDENTITY)))
    mapper.set_absolute_calibration(calibration(anchor=(0.4, 0.3, 1.0)))
    processor = ArmFrameProcessor(
        mapper,
        {"left": ArmSafetyFilter(xyz_min=[0, 0, 0], xyz_max=[0.2, 0.2, 0.2])},
        ["left"],
    )

    result = processor.switch_mapping_mode(
        "absolute", "PAUSED",
        {"left": ArmTarget([0, 0, 0], list(IDENTITY))},
        {"left": ArmTarget([0.4, 0.3, 1.0], list(IDENTITY))},
        ("session", "local-floor", 0),
    )

    assert result.accepted is False
    assert "outside configured workspace" in result.reason
    assert mapper.mode is MappingMode.RELATIVE
    assert mapper.is_calibrated("left") is True

from threading import get_ident

import pytest

from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmTarget
from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmMapper
from stardust_wuji_quest3_pc_retargeting.arm_control.absolute_session_calibration import AbsoluteCalibrationConfig, AbsoluteSessionCalibrator, CalibrationState
from stardust_wuji_quest3_pc_retargeting.protocol.validation import validate_tracking_frame
from stardust_wuji_quest3_pc_retargeting.runtime.arm_control_loop import ArmControlLoop, ArmFrameProcessor, LoopState
from stardust_wuji_quest3_pc_retargeting.runtime.latest_tracking import LatestTrackingBuffer
from stardust_wuji_quest3_pc_retargeting.safety.arm_safety_filter import ArmSafetyFilter


class FakeClock:
    def __init__(self, value=1_000_000_000):
        self.value = value

    def __call__(self):
        return self.value


class Adapter:
    def __init__(self, clock=None, delay_ns=0):
        self.clock = clock
        self.delay_ns = delay_ns
        self.calls = []
        self.thread_ids = []

    def initialize(self):
        self.thread_ids.append(get_ident())

    def send_targets(self, targets):
        self.thread_ids.append(get_ident())
        self.calls.append(targets)
        if self.clock:
            self.clock.value += self.delay_ns

    def close(self):
        self.thread_ids.append(get_ident())

    def get_desired_poses(self, frame="chassis"):
        return {"left": ArmTarget([0.4, 0.3, 1], [0, 0, 0, 1]), "right": ArmTarget([0.4, -0.3, 1], [0, 0, 0, 1])}

    def get_current_poses(self, frame="chassis"):
        return self.get_desired_poses(frame)


def arm_target(value):
    return {"left": ArmTarget([value, 0, 0], [0, 0, 0, 1])}


def test_latest_buffer_overwrites_1000_frames_without_queue_growth():
    buffer = LatestTrackingBuffer()
    for seq in range(1000):
        buffer.publish({"seq": seq}, receive_time_ns=seq)

    assert buffer.size == 1
    assert buffer.published_count == 1000
    assert buffer.snapshot().frame["seq"] == 999


def test_loop_runs_output_cycles_independent_of_new_input_and_uses_latest_only():
    clock = FakeClock()
    adapter = Adapter()
    seen = []
    buffer = LatestTrackingBuffer()
    buffer.publish({"seq": 1}, clock.value)
    buffer.publish({"seq": 2}, clock.value)
    loop = ArmControlLoop(adapter, buffer, lambda frame, dt, received: seen.append(frame["seq"]) or arm_target(frame["seq"]), clock_ns=clock)
    adapter.initialize()

    for _ in range(5):
        loop.tick(clock.value)
        clock.value += 10_000_000

    assert seen == [2]
    assert len(adapter.calls) == 5
    assert loop.stats.consumed_frames == 1
    assert loop.stats.sent_cycles == 5


def test_republished_duplicate_sequence_is_not_remapped():
    clock = FakeClock()
    adapter = Adapter()
    seen = []
    buffer = LatestTrackingBuffer()
    loop = ArmControlLoop(
        adapter,
        buffer,
        lambda frame, dt, received: seen.append(frame["seq"]) or arm_target(frame["seq"]),
        clock_ns=clock,
    )
    adapter.initialize()
    buffer.publish({"xr_session_id": "s", "seq": 4}, clock.value)
    loop.tick(clock.value)
    clock.value += 10_000_000
    buffer.publish({"xr_session_id": "s", "seq": 4}, clock.value)
    loop.tick(clock.value)

    assert seen == [4]


def test_loop_records_target_speed_against_actual_tick_time():
    clock = FakeClock()
    adapter = Adapter()
    buffer = LatestTrackingBuffer()
    buffer.publish({"xr_session_id": "s", "seq": 1}, clock.value)
    loop = ArmControlLoop(adapter, buffer, lambda frame, dt, received: arm_target(frame["seq"] * 0.001), clock_ns=clock)
    adapter.initialize()
    loop.tick(clock.value)
    clock.value += 10_000_000
    buffer.publish({"xr_session_id": "s", "seq": 2}, clock.value)
    loop.tick(clock.value)

    assert loop.stats.target_linear_speeds_mps[-1] == pytest.approx(0.1)


def test_fresh_hold_then_stale_pause_uses_local_receive_time():
    clock = FakeClock()
    adapter = Adapter()
    buffer = LatestTrackingBuffer()
    buffer.publish({"client_time_sec": -999999}, clock.value)
    loop = ArmControlLoop(adapter, buffer, lambda frame, dt, received: arm_target(1), clock_ns=clock)
    adapter.initialize()

    assert loop.tick(clock.value) is LoopState.ACTIVE
    clock.value += 60_000_000
    assert loop.tick(clock.value) is LoopState.HOLD
    clock.value += 50_000_000
    assert loop.tick(clock.value) is LoopState.PAUSED


def test_frame_just_over_disable_timeout_pauses_immediately():
    clock = FakeClock()
    adapter = Adapter()
    buffer = LatestTrackingBuffer()
    buffer.publish({"seq": 1}, clock.value)
    loop = ArmControlLoop(adapter, buffer, lambda frame, dt, received: arm_target(1), clock_ns=clock)
    adapter.initialize()
    assert loop.tick(clock.value) is LoopState.ACTIVE

    clock.value += 100_000_001

    assert loop.tick(clock.value) is LoopState.PAUSED
    assert loop.stats.stale_pauses == 1


def test_repeated_blocking_sdk_calls_record_misses_and_fault():
    clock = FakeClock()
    adapter = Adapter(clock, delay_ns=25_000_000)
    buffer = LatestTrackingBuffer()
    buffer.publish({"seq": 1}, clock.value)
    loop = ArmControlLoop(
        adapter, buffer, lambda frame, dt, received: arm_target(1), clock_ns=clock,
        sdk_block_fault_sec=0.02, consecutive_deadline_fault_count=2,
    )
    adapter.initialize()

    loop.tick(clock.value)
    buffer.publish({"seq": 2}, clock.value)
    loop.tick(clock.value)

    assert loop.state is LoopState.FAULT
    assert loop.stats.missed_deadlines == 2
    assert "blocked" in loop.fault_reason


def test_invalid_frame_processor_fails_closed_without_sending_candidate():
    clock = FakeClock()
    adapter = Adapter()
    buffer = LatestTrackingBuffer()
    buffer.publish({"session": {"active": False}}, clock.value)

    def reject_inactive(frame, dt, received):
        if not frame["session"]["active"]:
            raise ValueError("XR session inactive")
        return arm_target(1)

    loop = ArmControlLoop(adapter, buffer, reject_inactive, clock_ns=clock)
    adapter.initialize()

    assert loop.tick(clock.value) is LoopState.FAULT
    assert adapter.calls == []
    assert "XR session inactive" in loop.fault_reason


def tracking_frame(*, seq=1, left_valid=True, active=True, hmd_valid=True):
    hand = {
        "valid": left_valid,
        "joint_names": ["wrist"] if left_valid else [],
        "positions": [[0, 0, 0]] if left_valid else [],
        "orientations_xyzw": [[0, 0, 0, 1]] if left_valid else [],
    }
    invalid_hand = {"valid": False, "joint_names": [], "positions": [], "orientations_xyzw": []}
    return validate_tracking_frame({
        "schema": "quest3_web_teleop.v1", "type": "tracking_frame", "seq": seq,
        "client_time_sec": -9999, "xr_session_id": "session",
        "hmd": {"valid": hmd_valid, "position": [0, 1.6, 0], "orientation_xyzw": [0, 0, 0, 1]},
        "hands": {"left": hand, "right": invalid_hand},
        "session": {"active": active, "reference_space": "local-floor", "reference_space_revision": 0},
    })


def test_integrated_processor_holds_lost_hand_and_pauses_inactive_session():
    mapper = ArmMapper()
    mapper.engage("left", ArmTarget([0, 0, 0], [0, 0, 0, 1]), ArmTarget([0.4, 0.3, 1], [0, 0, 0, 1]))
    processor = ArmFrameProcessor(mapper, {"left": ArmSafetyFilter(max_input_position_jump_m=10)}, ["left"])
    clock = FakeClock()
    adapter = Adapter()
    buffer = LatestTrackingBuffer()
    loop = ArmControlLoop(adapter, buffer, processor, clock_ns=clock)
    adapter.initialize()

    buffer.publish(tracking_frame(), clock.value)
    assert loop.tick(clock.value) is LoopState.ACTIVE
    assert len(adapter.calls) == 1
    clock.value += 10_000_000
    buffer.publish(tracking_frame(seq=2, left_valid=False), clock.value)
    assert loop.tick(clock.value) is LoopState.HOLD
    assert len(adapter.calls) == 2
    clock.value += 10_000_000
    buffer.publish(tracking_frame(seq=3, active=False), clock.value)
    assert loop.tick(clock.value) is LoopState.PAUSED
    assert len(adapter.calls) == 2

    clock.value += 10_000_000
    buffer.publish(tracking_frame(seq=4, active=True), clock.value)
    assert loop.tick(clock.value) is LoopState.PAUSED
    assert len(adapter.calls) == 2


def test_calibration_reads_robot_on_loop_thread_and_never_sends_targets():
    clock = FakeClock()
    adapter = Adapter()
    calibrator = AbsoluteSessionCalibrator(
        AbsoluteCalibrationConfig(countdown_sec=0, sample_duration_sec=0.02, minimum_valid_samples=3)
    )
    mapper = ArmMapper(mapping_mode="absolute")
    processor = ArmFrameProcessor(
        mapper,
        {"left": ArmSafetyFilter(max_input_position_jump_m=10)},
        ["left"],
        adapter=adapter,
        calibrator=calibrator,
    )
    processor.start_absolute_calibration(clock.value)
    buffer = LatestTrackingBuffer()
    loop = ArmControlLoop(adapter, buffer, processor, clock_ns=clock)
    adapter.initialize()

    for seq in range(3):
        frame = tracking_frame()
        frame.seq = seq
        buffer.publish(frame, clock.value)
        assert loop.tick(clock.value) is LoopState.PAUSED
        clock.value += 10_000_000

    assert calibrator.state is CalibrationState.VALID
    assert mapper.is_calibrated("left") is True
    assert adapter.calls == []


def test_hmd_loss_cancels_partial_calibration():
    clock = FakeClock()
    adapter = Adapter()
    calibrator = AbsoluteSessionCalibrator(
        AbsoluteCalibrationConfig(countdown_sec=0, sample_duration_sec=1, minimum_valid_samples=3)
    )
    mapper = ArmMapper(mapping_mode="absolute")
    processor = ArmFrameProcessor(
        mapper,
        {"left": ArmSafetyFilter()},
        ["left"],
        adapter=adapter,
        calibrator=calibrator,
    )
    processor.start_absolute_calibration(clock.value)
    buffer = LatestTrackingBuffer()
    loop = ArmControlLoop(adapter, buffer, processor, clock_ns=clock)
    adapter.initialize()
    buffer.publish(tracking_frame(), clock.value)
    loop.tick(clock.value)
    clock.value += 10_000_000
    buffer.publish(tracking_frame(seq=2, hmd_valid=False), clock.value)

    assert loop.tick(clock.value) is LoopState.PAUSED
    assert calibrator.state is CalibrationState.INVALID
    assert calibrator.result is None

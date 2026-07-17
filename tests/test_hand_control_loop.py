import numpy as np

from stardust_wuji_quest3_pc_retargeting.hand_control.command_bridge import DryRunHandCommandSink
from stardust_wuji_quest3_pc_retargeting.hand_control.control_loop import HandControlLoop
from stardust_wuji_quest3_pc_retargeting.protocol.validation import validate_tracking_frame
from stardust_wuji_quest3_pc_retargeting.runtime.latest_tracking import LatestTrackingBuffer
from stardust_wuji_quest3_pc_retargeting.safety.hand_safety_filter import HandSafetyFilter
from stardust_wuji_quest3_pc_retargeting.sim.mock_webxr_sender import build_mock_frame


class Converter:
    def convert(self, hand):
        return np.asarray(hand.positions[:21], dtype=float)


class Retargeter:
    def retarget(self, mp21):
        return np.full(20, 0.4)


def test_formal_hand_loop_publishes_bimanual_safe_targets_and_pause_hold():
    buffer = LatestTrackingBuffer()
    sink = DryRunHandCommandSink()
    running = {"value": True}
    loop = HandControlLoop(
        tracking_buffer=buffer,
        converters={"left": Converter(), "right": Converter()},
        retargeters={"left": Retargeter(), "right": Retargeter()},
        filters={
            "left": HandSafetyFilter(lower=[0.0] * 20, upper=[1.0] * 20),
            "right": HandSafetyFilter(lower=[0.0] * 20, upper=[1.0] * 20),
        },
        sink=sink,
        running_provider=lambda: running["value"],
        state_provider=lambda: "RUNNING" if running["value"] else "PAUSED",
    )
    first = validate_tracking_frame(build_mock_frame(1))
    buffer.publish(first, receive_time_ns=1_000_000_000)

    active = loop.tick(1_010_000_000)

    assert active.hands["left"].enabled is True
    assert active.hands["right"].enabled is True
    np.testing.assert_allclose(active.hands["left"].safe_qpos, np.full(20, 0.4))

    running["value"] = False
    second = validate_tracking_frame(build_mock_frame(2))
    buffer.publish(second, receive_time_ns=1_020_000_000)
    paused = loop.tick(1_030_000_000)

    assert paused.teleop_state == "PAUSED"
    assert paused.hands["left"].enabled is False
    assert paused.hands["left"].safety_state == "HOLD"
    np.testing.assert_allclose(paused.hands["left"].safe_qpos, np.full(20, 0.4))


def test_formal_hand_loop_emits_one_stale_disabled_frame():
    buffer = LatestTrackingBuffer()
    sink = DryRunHandCommandSink()
    loop = HandControlLoop(
        tracking_buffer=buffer,
        converters={"left": Converter(), "right": Converter()},
        retargeters={"left": Retargeter(), "right": Retargeter()},
        filters={
            "left": HandSafetyFilter(lower=[0.0] * 20, upper=[1.0] * 20),
            "right": HandSafetyFilter(lower=[0.0] * 20, upper=[1.0] * 20),
        },
        sink=sink,
        running_provider=lambda: True,
        state_provider=lambda: "RUNNING",
        stale_emit_sec=0.2,
    )
    tracking = validate_tracking_frame(build_mock_frame(1))
    buffer.publish(tracking, receive_time_ns=1_000_000_000)
    loop.tick(1_010_000_000)

    stale = loop.tick(1_250_000_000)

    assert stale.hands["left"].enabled is False
    assert stale.hands["left"].safety_state == "DISABLED"
    assert loop.tick(1_260_000_000) is None

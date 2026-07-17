import numpy as np
import pytest
import socket

from stardust_wuji_quest3_pc_retargeting.hand_control.command_bridge import (
    DryRunHandCommandSink,
    HandBridgeFrame,
    HandBridgeSide,
    UdpHandCommandSink,
)
from stardust_wuji_quest3_pc_retargeting.hand_control.ros2_bridge_core import Ros2BridgeCore


def side(enabled=True):
    return HandBridgeSide(
        valid=True,
        mp21=np.zeros((21, 3)).tolist(),
        raw_qpos=np.linspace(0.0, 1.0, 20).tolist(),
        safe_qpos=np.linspace(0.0, 0.5, 20).tolist(),
        enabled=enabled,
        safety_state="ACTIVE" if enabled else "HOLD",
    )


def frame(seq=1, session="s"):
    return HandBridgeFrame(
        seq=seq,
        client_time_sec=1.25,
        receive_time_ns=123,
        xr_session_id=session,
        teleop_state="RUNNING",
        hands={"left": side(), "right": side()},
    )


def test_hand_bridge_frame_round_trips_compact_json():
    restored = HandBridgeFrame.from_json_bytes(frame().to_json_bytes())

    assert restored.seq == 1
    assert restored.hands["left"].enabled is True
    assert len(restored.hands["right"].safe_qpos) == 20


def test_hand_bridge_rejects_wrong_shapes_and_nonfinite_values():
    bad = side()
    object.__setattr__(bad, "safe_qpos", [float("nan")] * 20)

    with pytest.raises(ValueError, match="safe_qpos"):
        bad.validate()


def test_dryrun_sink_records_latest_frame_without_hardware():
    sink = DryRunHandCommandSink(history_size=2)

    for seq in range(3):
        sink.publish(frame(seq))

    assert sink.publish_count == 3
    assert sink.snapshot().seq == 2
    assert [item.seq for item in sink.history] == [1, 2]


def test_ros2_bridge_core_rejects_duplicate_and_out_of_order_frames():
    core = Ros2BridgeCore(command_timeout_sec=0.25)

    assert core.ingest(frame(2).to_json_bytes(), now_monotonic=1.0).seq == 2
    assert core.ingest(frame(2).to_json_bytes(), now_monotonic=1.1) is None
    assert core.ingest(frame(1).to_json_bytes(), now_monotonic=1.2) is None
    assert core.ingest(frame(0, session="new").to_json_bytes(), now_monotonic=1.3).seq == 0
    assert core.stats.duplicate_frames == 1
    assert core.stats.out_of_order_frames == 1
    assert core.stale(now_monotonic=1.54) is False
    assert core.stale(now_monotonic=1.56) is True


def test_udp_hand_sink_delivers_frame_to_local_bridge_socket():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(1.0)
    port = receiver.getsockname()[1]
    sink = UdpHandCommandSink("127.0.0.1", port)
    try:
        sink.publish(frame(7))
        payload, _peer = receiver.recvfrom(65_535)
        restored = HandBridgeFrame.from_json_bytes(payload)
        assert restored.seq == 7
        assert sink.publish_count == 1
    finally:
        sink.close()
        receiver.close()

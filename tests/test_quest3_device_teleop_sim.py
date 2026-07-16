import asyncio
import json
import socket
import time

import numpy as np
import pytest
import websockets

from example.input_devices.quest3_device import Quest3Device
from stardust_wuji_quest3_pc_retargeting.conversion.hand_joint_names import WEBXR_HAND_JOINT_NAMES
from stardust_wuji_quest3_pc_retargeting.protocol.messages import SCHEMA


def frame(left_valid=True, right_valid=True):
    positions = [[float(i), 0.0, 0.0] for i, _ in enumerate(WEBXR_HAND_JOINT_NAMES)]

    def hand(valid):
        return {
            "valid": valid,
            "joint_names": WEBXR_HAND_JOINT_NAMES,
            "positions": positions,
            "orientations_xyzw": [[0.0, 0.0, 0.0, 1.0]] * len(WEBXR_HAND_JOINT_NAMES),
        }

    return {
        "schema": SCHEMA,
        "type": "tracking_frame",
        "seq": 42,
        "client_time_sec": 1.0,
        "xr_session_id": "test",
        "hmd": {"valid": True, "position": [0.0, 1.6, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "hands": {"left": hand(left_valid), "right": hand(right_valid)},
        "session": {"active": True, "visibility": "visible", "reference_space": "local-floor"},
    }


def unused_port():
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def test_constructor_matches_teleop_sim_and_returns_expected_keys():
    device = Quest3Device(host="127.0.0.1", port=unused_port(), start_server=False)

    data = device.get_fingers_data()

    assert set(data) >= {"left_fingers", "right_fingers"}
    np.testing.assert_allclose(data["left_fingers"], np.zeros((21, 3)))
    assert device.get_controller_state().deadman is False
    assert device.get_frame_age_sec() is None


def test_ingested_frame_outputs_mp21_and_zeroes_invalid_hand():
    device = Quest3Device(host="127.0.0.1", port=unused_port(), start_server=False)

    device.ingest_payload(frame(left_valid=True, right_valid=False))
    data = device.get_fingers_data()

    assert data["left_fingers"].shape == (21, 3)
    assert data["right_fingers"].shape == (21, 3)
    assert np.count_nonzero(data["left_fingers"]) > 0
    assert np.count_nonzero(data["right_fingers"]) == 0
    assert data["left"] is data["left_fingers"]
    assert data["right"] is data["right_fingers"]
    assert device.get_controller_state().deadman is True
    assert device.get_frame_age_sec() >= 0.0


def test_stale_frame_zeroes_both_hands():
    device = Quest3Device(host="127.0.0.1", port=unused_port(), stale_timeout_sec=0.01, start_server=False)

    device.ingest_payload(frame())
    time.sleep(0.03)

    data = device.get_fingers_data()
    assert np.count_nonzero(data["left_fingers"]) == 0
    assert np.count_nonzero(data["right_fingers"]) == 0
    assert device.get_controller_state().deadman is False


def test_from_service_config_reads_legacy_top_level_config(tmp_path):
    cfg = tmp_path / "control_pc.yaml"
    cfg.write_text(
        "quest_host: 127.0.0.1\n"
        f"quest_port: {unused_port()}\n"
        "stale_timeout_sec: 0.5\n"
        "grip_deadman_threshold: 0.75\n",
        encoding="utf-8",
    )

    device = Quest3Device.from_service_config(cfg, start_server=False)

    assert device.host == "127.0.0.1"
    assert device.port > 0
    assert device.stale_timeout_sec == 0.5
    assert device.grip_deadman_threshold == 0.75


def test_background_websocket_ingests_tracking_frame():
    port = unused_port()
    device = Quest3Device(host="127.0.0.1", port=port, start_server=True)

    async def send_frame():
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await ws.send(json.dumps({"schema": SCHEMA, "type": "hello"}))
            await ws.send(json.dumps(frame()))

    try:
        asyncio.run(send_frame())
        deadline = time.time() + 1.0
        while device.get_frame_age_sec() is None and time.time() < deadline:
            time.sleep(0.01)
        data = device.get_fingers_data()
        assert np.count_nonzero(data["left_fingers"]) > 0
        assert np.count_nonzero(data["right_fingers"]) > 0
    finally:
        device.close()


def test_teleop_sim_quest3_defaults_use_webxr_mapping(monkeypatch):
    from example import teleop_sim

    captured = {}

    class FakeQuest3Device:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(teleop_sim, "Quest3Device", FakeQuest3Device)

    teleop_sim.create_input_device("quest3", hand_side="left")

    assert captured["left_config"].endswith("configs/quest3_web/webxr_hand_mapping_left.yaml")
    assert captured["right_config"].endswith("configs/quest3_web/webxr_hand_mapping_right.yaml")


def test_teleop_real_quest3_defaults_use_webxr_mapping(monkeypatch):
    from example import teleop_real

    captured = {}

    class FakeQuest3Device:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(teleop_real, "Quest3Device", FakeQuest3Device)

    teleop_real.create_input_device("quest3", hand_side="left")

    assert captured["left_config"].endswith("configs/quest3_web/webxr_hand_mapping_left.yaml")
    assert captured["right_config"].endswith("configs/quest3_web/webxr_hand_mapping_right.yaml")


def test_teleop_sim_can_create_real_quest3_device_from_project_root():
    from example.teleop_sim import create_input_device

    device = create_input_device("quest3", hand_side="left", quest_host="127.0.0.1", quest_port=unused_port())

    try:
        assert isinstance(device, Quest3Device)
        assert set(device.get_fingers_data()) >= {"left_fingers", "right_fingers"}
    finally:
        device.close()

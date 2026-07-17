import json
import math

import pytest

from stardust_wuji_quest3_pc_retargeting.protocol.json_codec import decode_message, encode_message
from stardust_wuji_quest3_pc_retargeting.protocol.messages import SCHEMA, TrackingFrame
from stardust_wuji_quest3_pc_retargeting.protocol.validation import ProtocolError, validate_tracking_frame


def frame_payload():
    names = ["wrist", "thumb-metacarpal", "thumb-phalanx-proximal"]
    positions = [[0.0, 1.0, 2.0], [0.1, 1.1, 2.1], [0.2, 1.2, 2.2]]
    orientations = [[0.0, 0.0, 0.0, 1.0]] * 3
    return {
        "schema": SCHEMA,
        "type": "tracking_frame",
        "seq": 7,
        "client_time_sec": 12.5,
        "xr_session_id": "session-a",
        "hmd": {
            "valid": True,
            "position": [0.0, 1.6, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "hands": {
            "left": {
                "valid": True,
                "joint_names": names,
                "positions": positions,
                "orientations_xyzw": orientations,
            },
            "right": {
                "valid": False,
                "joint_names": [],
                "positions": [],
                "orientations_xyzw": [],
            },
        },
        "session": {
            "active": True,
            "visibility": "visible",
            "reference_space": "local-floor",
            "reference_space_revision": 3,
        },
    }


def test_valid_tracking_frame_becomes_dataclass():
    frame = validate_tracking_frame(frame_payload())

    assert isinstance(frame, TrackingFrame)
    assert frame.seq == 7
    assert frame.hands["left"].valid is True
    assert frame.hands["right"].joint_names == []
    assert frame.session.reference_space_revision == 3
    assert frame.arm_wrists["left"].valid is True
    assert frame.arm_wrists["right"].valid is False


def test_arm_wrist_channel_is_independent_from_full_hand_validity():
    payload = frame_payload()
    payload["hands"]["left"] = {
        "valid": False,
        "joint_names": [],
        "positions": [],
        "orientations_xyzw": [],
    }
    payload["arm_wrists"] = {
        "left": {
            "valid": True,
            "position": [0.1, 0.2, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "right": {
            "valid": False,
            "position": [0.0, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
    }

    frame = validate_tracking_frame(payload)

    assert frame.hands["left"].valid is False
    assert frame.arm_wrists["left"].valid is True
    assert frame.arm_wrists["left"].position == [0.1, 0.2, 0.3]


def test_old_client_is_relative_compatible_but_absolute_revision_is_missing():
    payload = frame_payload()
    payload["session"].pop("reference_space_revision")

    frame = validate_tracking_frame(payload)

    assert frame.session.reference_space_revision is None


def test_invalid_reference_space_revision_is_rejected():
    payload = frame_payload()
    payload["session"]["reference_space_revision"] = -1

    with pytest.raises(ProtocolError, match="reference_space_revision"):
        validate_tracking_frame(payload)


def test_json_round_trip_preserves_schema_and_type():
    frame = decode_message(json.dumps(frame_payload()))
    encoded = json.loads(encode_message(frame))

    assert encoded["schema"] == SCHEMA
    assert encoded["type"] == "tracking_frame"
    assert encoded["hands"]["left"]["joint_names"][0] == "wrist"
    assert encoded["arm_wrists"]["left"]["valid"] is True


@pytest.mark.parametrize(
    "mutator",
    [
        lambda p: p.update({"schema": "wrong"}),
        lambda p: p.update({"type": "status"}),
        lambda p: p["hands"]["left"].update({"positions": [[math.nan, 0.0, 0.0]] * 3}),
        lambda p: p["hands"]["left"].update({"orientations_xyzw": []}),
        lambda p: p["hmd"].update({"orientation_xyzw": [0.0, 0.0, 0.0]}),
    ],
)
def test_invalid_tracking_frame_is_rejected(mutator):
    payload = frame_payload()
    mutator(payload)

    with pytest.raises(ProtocolError):
        validate_tracking_frame(payload)

import numpy as np
import pytest

from stardust_wuji_quest3_pc_retargeting.conversion.hand_joint_names import WEBXR_HAND_JOINT_NAMES
from stardust_wuji_quest3_pc_retargeting.conversion.webxr_to_mp21 import WebXRToMP21Converter
from stardust_wuji_quest3_pc_retargeting.protocol.validation import ProtocolError, validate_hand


def webxr_hand(valid=True):
    positions = [[float(i), float(i + 1), float(i + 2)] for i, _ in enumerate(WEBXR_HAND_JOINT_NAMES)]
    return validate_hand(
        {
            "valid": valid,
            "joint_names": WEBXR_HAND_JOINT_NAMES,
            "positions": positions,
            "orientations_xyzw": [[0.0, 0.0, 0.0, 1.0]] * len(WEBXR_HAND_JOINT_NAMES),
        },
        "left",
    )


def test_converter_outputs_wrist_relative_mp21_shape():
    points = WebXRToMP21Converter().convert(webxr_hand())

    assert points.shape == (21, 3)
    np.testing.assert_allclose(points[0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(points[8], [9.0, 9.0, 9.0])


def test_invalid_hand_returns_zero_mp21():
    points = WebXRToMP21Converter().convert(webxr_hand(valid=False))

    assert points.shape == (21, 3)
    assert np.count_nonzero(points) == 0


def test_missing_required_joint_raises_protocol_error():
    hand = webxr_hand()
    hand.joint_names.remove("index-finger-tip")

    with pytest.raises(ProtocolError):
        WebXRToMP21Converter().convert(hand)


def test_converter_scale_can_be_loaded_from_yaml(tmp_path):
    cfg = tmp_path / "mapping.yaml"
    cfg.write_text("scale: 2.0\nwrist_relative: true\n", encoding="utf-8")

    points = WebXRToMP21Converter.from_yaml(cfg).convert(webxr_hand())

    np.testing.assert_allclose(points[8], [18.0, 18.0, 18.0])


def test_legacy_quest26_yaml_accepts_webxr_joint_names():
    converter = WebXRToMP21Converter.from_yaml("configs/quest3/quest26_to_mp21_left.yaml")

    points = converter.convert(webxr_hand())

    assert points.shape == (21, 3)
    np.testing.assert_allclose(points[8], [9.0, 9.0, 9.0])

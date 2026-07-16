import numpy as np
from pathlib import Path

from stardust_wuji_quest3_pc_retargeting.safety import HandSafetyFilter, SafetyState


def test_hand_filter_clips_limits_and_rate_limits_jump():
    filt = HandSafetyFilter(lower=[0.0] * 20, upper=[1.0] * 20, max_delta=0.2)

    first = filt.filter(np.full(20, 0.5), valid=True, now_sec=0.0, running=True)
    second = filt.filter(np.full(20, 2.0), valid=True, now_sec=0.01, running=True)

    assert first.enabled is True
    np.testing.assert_allclose(second.qpos, np.full(20, 0.7))


def test_hand_filter_holds_on_invalid_or_stale():
    filt = HandSafetyFilter(lower=[0.0] * 20, upper=[1.0] * 20, max_delta=0.5, stale_timeout_sec=0.1)
    filt.filter(np.full(20, 0.4), valid=True, now_sec=0.0, running=True)

    invalid = filt.filter(np.full(20, 0.8), valid=False, now_sec=0.05, running=True)
    stale = filt.filter(np.full(20, 0.8), valid=True, now_sec=0.2, running=True)

    np.testing.assert_allclose(invalid.qpos, np.full(20, 0.4))
    assert stale.enabled is False
    np.testing.assert_allclose(stale.qpos, np.full(20, 0.4))


def test_hand_filter_loads_yaml_and_supports_real_teleop_api():
    filt = HandSafetyFilter.from_yaml("configs/safety/wh110_left.yaml")

    result = filt.filter(
        np.full(20, 0.5),
        frame_age_sec=0.01,
        deadman=True,
        tracking_valid=True,
    )

    assert result.state == SafetyState.ACTIVE
    assert result.enabled is True
    np.testing.assert_allclose(result.qpos, np.full(20, 0.5))


def test_hand_filter_safe_open_on_deadman_release_can_be_sent():
    filt = HandSafetyFilter.from_yaml(
        "configs/safety/wh110_left.yaml",
        safe_open_on_deadman_release=True,
    )

    result = filt.filter(
        np.full(20, 0.5),
        frame_age_sec=0.01,
        deadman=False,
        tracking_valid=True,
    )

    assert result.state == SafetyState.SAFE_OPEN
    assert result.enabled is True
    np.testing.assert_allclose(result.qpos, np.zeros(20))


def test_teleop_real_sends_active_and_safe_open_states():
    teleop_real = Path("example/teleop_real.py").read_text(encoding="utf-8")

    assert "SafetyState.ACTIVE.value" in teleop_real
    assert "SafetyState.SAFE_OPEN.value" in teleop_real


def test_teleop_real_safety_output_stays_numpy_array():
    from example.teleop_real import evaluate_retarget_and_safety

    class InputDevice:
        def get_controller_state(self):
            return type("Controller", (), {"deadman": True})()

        def get_frame_age_sec(self):
            return 0.01

    class Retargeter:
        def retarget(self, fingers_pose):
            return np.full(20, 0.5)

    filt = HandSafetyFilter.from_yaml("configs/safety/wh110_left.yaml")

    result = evaluate_retarget_and_safety(
        fingers_pose=np.ones((21, 3)),
        input_device=InputDevice(),
        retargeter=Retargeter(),
        safety_filter=filt,
    )

    assert isinstance(result.output_qpos, np.ndarray)
    assert result.output_qpos.shape == (20,)

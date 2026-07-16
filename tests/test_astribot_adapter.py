import builtins
from unittest.mock import Mock

import numpy as np
import pytest

from stardust_wuji_quest3_pc_retargeting.arm_control.arm_mapper import ArmTarget
from stardust_wuji_quest3_pc_retargeting.arm_control.astribot_adapter import AstribotAdapter


def pose(x):
    return ArmTarget([x, x + 1.0, x + 2.0], [0.0, 0.0, 0.0, 1.0])


def test_real_adapter_uses_official_signatures_and_batches_both_arms():
    robot = Mock()
    robot.arm_left_name = "left-sdk"
    robot.arm_right_name = "right-sdk"
    robot.chassis_frame_name = "base-sdk"
    robot.get_control_rights_status.return_value = True
    robot.astribot_interface.is_alive.return_value = True
    robot.astribot_interface.get_robot_mode.return_value = "safe"
    robot.get_desired_cartesian_pose.return_value = [pose(1).as_pose_list(), pose(2).as_pose_list()]
    robot.get_current_cartesian_pose.return_value = [pose(3).as_pose_list(), pose(4).as_pose_list()]
    factory = Mock(return_value=robot)
    adapter = AstribotAdapter(freq_hz=100.0, enable_real=True, robot_factory=factory)

    adapter.initialize()
    desired = adapter.get_desired_poses()
    current = adapter.get_current_poses()
    adapter.send_targets({"right": pose(8), "left": pose(7)})

    factory.assert_called_once_with(freq=100.0)
    robot.get_desired_cartesian_pose.assert_called_once_with(names=["left-sdk", "right-sdk"], frame="base-sdk")
    robot.get_current_cartesian_pose.assert_called_once_with(names=["left-sdk", "right-sdk"], frame="base-sdk")
    robot.set_cartesian_pose.assert_called_once_with(
        ["left-sdk", "right-sdk"],
        [pose(7).as_pose_list(), pose(8).as_pose_list()],
        control_way="filter",
        use_wbc=False,
        add_default_torso=False,
    )
    robot.astribot_interface.is_alive.assert_called_once_with()
    robot.astribot_interface.get_robot_mode.assert_called_once_with()
    np.testing.assert_allclose(desired["left"].position, pose(1).position)
    np.testing.assert_allclose(current["right"].position, pose(4).position)


def test_dry_run_never_imports_sdk_and_records_latest_batch(monkeypatch):
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name.startswith("astribot_sdk"):
            raise AssertionError("dry-run imported Astribot SDK")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    adapter = AstribotAdapter(enable_real=False)
    adapter.initialize()
    adapter.send_targets({"left": pose(1), "right": pose(2)})

    assert adapter.stats.send_calls == 1
    assert set(adapter.last_targets) == {"left", "right"}
    np.testing.assert_allclose(adapter.get_desired_poses()["right"].position, pose(2).position)


def test_competing_control_client_fails_initialization_closed():
    robot = Mock()
    robot.arm_left_name = "left"
    robot.arm_right_name = "right"
    robot.chassis_frame_name = "chassis"
    robot.get_control_rights_status.return_value = False
    robot.astribot_interface.is_alive.return_value = True
    robot.astribot_interface.get_robot_mode.return_value = "safe"
    robot.astribot_interface.shutdown = Mock()
    adapter = AstribotAdapter(enable_real=True, robot_factory=Mock(return_value=robot))

    with pytest.raises(RuntimeError, match="control rights unavailable"):
        adapter.initialize()

    robot.astribot_interface.shutdown.assert_called_once_with()


def test_real_adapter_rechecks_control_rights_before_every_send():
    robot = Mock()
    robot.arm_left_name = "left"
    robot.arm_right_name = "right"
    robot.chassis_frame_name = "chassis"
    robot.get_control_rights_status.side_effect = [True, False]
    robot.astribot_interface.is_alive.return_value = True
    robot.astribot_interface.get_robot_mode.return_value = "safe"
    adapter = AstribotAdapter(enable_real=True, robot_factory=Mock(return_value=robot))
    adapter.initialize()

    with pytest.raises(RuntimeError, match="control rights unavailable"):
        adapter.send_targets({"left": pose(1)})

    robot.set_cartesian_pose.assert_not_called()
    assert adapter.stats.send_calls == 0


def test_real_adapter_refuses_existing_control_rights_service_before_sdk_import(monkeypatch):
    completed = Mock(returncode=0, stdout="/astribot/control_rights\n", stderr="")
    monkeypatch.setattr(
        "stardust_wuji_quest3_pc_retargeting.arm_control.astribot_adapter.subprocess.run",
        Mock(return_value=completed),
    )
    adapter = AstribotAdapter(enable_real=True)

    with pytest.raises(RuntimeError, match="already owned"):
        adapter.initialize()

    assert adapter.initialized is False


def test_authorized_real_adapter_passes_high_control_rights_to_vendor_factory():
    robot = Mock()
    robot.arm_left_name = "left"
    robot.arm_right_name = "right"
    robot.chassis_frame_name = "chassis"
    robot.get_control_rights_status.return_value = True
    robot.astribot_interface.is_alive.return_value = True
    robot.astribot_interface.get_robot_mode.return_value = "safe"
    factory = Mock(return_value=robot)
    adapter = AstribotAdapter(enable_real=True, high_control_rights=True, robot_factory=factory)

    adapter.initialize()

    factory.assert_called_once_with(freq=100.0, high_control_rights=True)


def test_takeover_verification_ignores_prior_owner_service_client(monkeypatch):
    node_list = Mock(returncode=0, stdout="/web_astribot_2451\n", stderr="")
    node_info = Mock(
        returncode=0,
        stdout=(
            "Service Servers:\n"
            "  /web_astribot_2451/get_parameters: type\n"
            "Service Clients:\n"
            "  /astribot/control_rights: astribot_msgs/srv/RawRequest\n"
        ),
        stderr="",
    )
    run = Mock(side_effect=[node_list, node_info])
    monkeypatch.setattr(
        "stardust_wuji_quest3_pc_retargeting.arm_control.astribot_adapter.subprocess.run", run
    )
    adapter = AstribotAdapter(enable_real=True, high_control_rights=True)

    adapter._require_prior_web_owner_released()


def test_default_arm_adapter_forbids_implicit_torso_commanding():
    with pytest.raises(ValueError, match="default torso is forbidden"):
        AstribotAdapter(enable_real=True, add_default_torso=True)

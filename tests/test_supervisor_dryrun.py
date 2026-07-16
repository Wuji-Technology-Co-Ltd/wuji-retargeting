from stardust_wuji_quest3_pc_retargeting.conversion.hand_joint_names import WEBXR_HAND_JOINT_NAMES
from stardust_wuji_quest3_pc_retargeting.protocol.messages import SCHEMA
from stardust_wuji_quest3_pc_retargeting.runtime.supervisor import DryRunSupervisor


def frame():
    positions = [[float(i) * 0.01, 0.0, 0.0] for i, _ in enumerate(WEBXR_HAND_JOINT_NAMES)]
    hand = {
        "valid": True,
        "joint_names": WEBXR_HAND_JOINT_NAMES,
        "positions": positions,
        "orientations_xyzw": [[0.0, 0.0, 0.0, 1.0]] * len(WEBXR_HAND_JOINT_NAMES),
    }
    return {
        "schema": SCHEMA,
        "type": "tracking_frame",
        "seq": 1,
        "client_time_sec": 1.0,
        "hmd": {"valid": True, "position": [0.0, 1.6, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "hands": {"left": hand, "right": hand},
        "session": {"active": True, "visibility": "visible", "reference_space": "local-floor"},
    }


def test_supervisor_dryrun_produces_bimanual_outputs_after_start():
    supervisor = DryRunSupervisor()
    supervisor.handle_command("calibrate")
    supervisor.handle_command("start")

    output = supervisor.process_payload(frame())

    assert output.state == "RUNNING"
    assert set(output.hands) == {"left", "right"}
    assert set(output.arms) == {"left", "right"}
    assert len(output.hands["left"].qpos) == 20
    assert output.hands["left"].enabled is True
    assert output.arms["right"].enabled is True

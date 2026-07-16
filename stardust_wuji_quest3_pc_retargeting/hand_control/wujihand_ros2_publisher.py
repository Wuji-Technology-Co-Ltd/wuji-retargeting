from __future__ import annotations

from dataclasses import dataclass


JOINT_NAMES_NO_PREFIX = [
    "finger1_joint1", "finger1_joint2", "finger1_joint3", "finger1_joint4",
    "finger2_joint1", "finger2_joint2", "finger2_joint3", "finger2_joint4",
    "finger3_joint1", "finger3_joint2", "finger3_joint3", "finger3_joint4",
    "finger4_joint1", "finger4_joint2", "finger4_joint3", "finger4_joint4",
    "finger5_joint1", "finger5_joint2", "finger5_joint3", "finger5_joint4",
]


@dataclass
class DryRunJointState:
    name: list[str]
    position: list[float]


class WujiHandRos2Publisher:
    def __init__(self, hand_side: str, hand_name: str | None = None, enable_real: bool = False):
        if hand_side not in {"left", "right"}:
            raise ValueError("hand_side must be left or right")
        self.hand_side = hand_side
        self.hand_name = hand_name or f"{hand_side}_hand"
        self.enable_real = bool(enable_real)
        self.joint_names = [f"{hand_side}_{name}" for name in JOINT_NAMES_NO_PREFIX]
        self.last_dryrun: DryRunJointState | None = None
        self._publisher = None
        if self.enable_real:
            self._init_ros2()

    def _init_ros2(self) -> None:
        import rclpy  # noqa: F401
        from sensor_msgs.msg import JointState  # noqa: F401

        raise RuntimeError("create this publisher from an existing rclpy Node in the hardware integration path")

    def publish(self, qpos) -> DryRunJointState | None:
        qpos_list = [float(value) for value in qpos]
        if len(qpos_list) != 20:
            raise ValueError("qpos must contain 20 values")
        if not self.enable_real:
            self.last_dryrun = DryRunJointState(name=self.joint_names, position=qpos_list)
            return self.last_dryrun
        raise RuntimeError("real ROS2 publishing requires Node-bound hardware adapter")

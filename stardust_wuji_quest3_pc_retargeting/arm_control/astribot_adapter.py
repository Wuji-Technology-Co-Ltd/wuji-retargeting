from __future__ import annotations

from dataclasses import dataclass, field
from importlib import import_module
from pathlib import Path
import subprocess
import time
from typing import Callable, Mapping

from .arm_mapper import ArmTarget


@dataclass
class AdapterStats:
    send_calls: int = 0
    desired_read_calls: int = 0
    current_read_calls: int = 0


@dataclass
class AstribotAdapter:
    freq_hz: float = 100.0
    enable_real: bool = False
    control_way: str = "filter"
    use_wbc: bool = False
    add_default_torso: bool = False
    allow_whole_body: bool = False
    robot_factory: Callable[..., object] | None = None
    sdk_root: str | Path = "/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master"
    high_control_rights: bool = False
    takeover_owner_prefix: str = "/web_astribot_"
    stats: AdapterStats = field(default_factory=AdapterStats, init=False)

    def __post_init__(self) -> None:
        self.freq_hz = float(self.freq_hz)
        if self.freq_hz <= 0.0:
            raise ValueError("freq_hz must be positive")
        if self.control_way != "filter":
            raise ValueError("the first-stage arm adapter requires control_way='filter'")
        if self.use_wbc:
            raise ValueError("the first-stage arm adapter requires use_wbc=False")
        if self.add_default_torso and not self.allow_whole_body:
            raise ValueError("default torso is forbidden unless whole-body control is explicitly enabled")
        self._robot = None
        self._initialized = False
        self._closed = False
        self._names = {"left": "astribot_arm_left", "right": "astribot_arm_right"}
        self._chassis_frame = "chassis"
        identity = ArmTarget([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        self._dryrun_desired = {side: self._copy_target(identity) for side in self._names}
        self._dryrun_current = {side: self._copy_target(identity) for side in self._names}
        self.last_targets: dict[str, ArmTarget] = {}

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def chassis_frame_name(self) -> str:
        return self._chassis_frame

    @property
    def control_rights(self) -> bool:
        if not self._initialized:
            return False
        if not self.enable_real:
            return True
        status = getattr(self._robot, "get_control_rights_status", None)
        return bool(callable(status) and status())

    def initialize(self) -> None:
        if self._initialized:
            return
        if self._closed:
            raise RuntimeError("adapter is closed")
        if self.enable_real:
            if self.robot_factory is None and not self.high_control_rights:
                self._require_unowned_control_service()
            factory = self.robot_factory or self._load_robot_factory()
            from stardust_wuji_quest3_pc_retargeting.hardware_audit.m7_audit import refuse_vendor_force_takeover

            with refuse_vendor_force_takeover():
                factory_kwargs = {"freq": self.freq_hz}
                if self.high_control_rights:
                    factory_kwargs["high_control_rights"] = True
                self._robot = factory(**factory_kwargs)
            if self.high_control_rights and self.robot_factory is None:
                time.sleep(0.20)
                self._require_prior_web_owner_released()
            self._names = {
                "left": str(self._required_attribute("arm_left_name")),
                "right": str(self._required_attribute("arm_right_name")),
            }
            self._chassis_frame = str(self._required_attribute("chassis_frame_name"))
            try:
                self._require_real_safety(full_check=True)
            except RuntimeError:
                self.close()
                raise
        self._initialized = True

    def get_desired_poses(self, frame: str = "chassis") -> dict[str, ArmTarget]:
        self._require_initialized()
        self.stats.desired_read_calls += 1
        if not self.enable_real:
            return {side: self._copy_target(target) for side, target in self._dryrun_desired.items()}
        values = self._robot.get_desired_cartesian_pose(names=self._ordered_names(), frame=self._resolve_frame(frame))
        return self._parse_pose_response(values, "desired")

    def get_current_poses(self, frame: str = "chassis") -> dict[str, ArmTarget]:
        self._require_initialized()
        self.stats.current_read_calls += 1
        if not self.enable_real:
            return {side: self._copy_target(target) for side, target in self._dryrun_current.items()}
        values = self._robot.get_current_cartesian_pose(names=self._ordered_names(), frame=self._resolve_frame(frame))
        return self._parse_pose_response(values, "current")

    def set_dryrun_poses(
        self,
        *,
        desired: Mapping[str, ArmTarget] | None = None,
        current: Mapping[str, ArmTarget] | None = None,
    ) -> None:
        if self.enable_real:
            raise RuntimeError("dry-run poses cannot be set in real mode")
        if desired is not None:
            self._dryrun_desired = self._validated_targets(desired)
        if current is not None:
            self._dryrun_current = self._validated_targets(current)

    def send_targets(self, targets: Mapping[str, ArmTarget]) -> None:
        self._require_initialized()
        validated = self._validated_targets(targets, allow_subset=True)
        if not validated:
            raise ValueError("at least one arm target is required")
        sides = [side for side in ("left", "right") if side in validated]
        names = [self._names[side] for side in sides]
        if any(name not in {self._names["left"], self._names["right"]} for name in names):
            raise RuntimeError("arm adapter command batch contains a non-arm name")
        poses = [validated[side].as_pose_list() for side in sides]
        if self.enable_real:
            self._require_real_safety(full_check=False)
        self.stats.send_calls += 1
        self.last_targets = {side: self._copy_target(validated[side]) for side in sides}
        if not self.enable_real:
            self._dryrun_desired.update(self.last_targets)
            return
        self._robot.set_cartesian_pose(
            names,
            poses,
            control_way=self.control_way,
            use_wbc=False,
            add_default_torso=self.add_default_torso,
        )

    def close(self) -> None:
        if self._closed:
            return
        robot = self._robot
        self._robot = None
        self._initialized = False
        self._closed = True
        if robot is not None:
            interface = getattr(robot, "astribot_interface", None)
            shutdown = getattr(interface, "shutdown", None)
            if callable(shutdown):
                shutdown()

    def _load_robot_factory(self):
        from stardust_wuji_quest3_pc_retargeting.hardware_audit.m7_audit import prepare_vendor_robotics_import

        prepare_vendor_robotics_import(self.sdk_root)
        module = import_module("astribot_sdk.core.astribot_api.astribot_client")
        return module.Astribot

    def _required_attribute(self, name: str):
        if self._robot is None or not hasattr(self._robot, name):
            raise RuntimeError(f"Astribot SDK object is missing {name}")
        return getattr(self._robot, name)

    def _require_real_safety(self, full_check: bool) -> None:
        status = getattr(self._robot, "get_control_rights_status", None)
        if not callable(status) or not bool(status()):
            raise RuntimeError("Astribot control rights unavailable; another SDK/control client may be active")
        if not full_check:
            return
        interface = getattr(self._robot, "astribot_interface", None)
        is_alive = getattr(interface, "is_alive", None)
        get_mode = getattr(interface, "get_robot_mode", None)
        if not callable(is_alive) or not bool(is_alive()):
            raise RuntimeError("Astribot interface is not alive")
        if not callable(get_mode) or get_mode() != "safe":
            raise RuntimeError("Astribot robot mode must be safe")

    @staticmethod
    def _require_unowned_control_service() -> None:
        try:
            completed = subprocess.run(
                ["ros2", "service", "list"],
                capture_output=True,
                text=True,
                timeout=5.0,
                check=False,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f"cannot verify Astribot control-rights ownership: {exc}") from exc
        if completed.returncode != 0:
            raise RuntimeError("cannot verify Astribot control-rights ownership from the ROS graph")
        services = {line.strip() for line in completed.stdout.splitlines()}
        if "/astribot/control_rights" in services:
            raise RuntimeError(
                "Astribot control rights are already owned; stop WebUI/VR/SDK controllers before M8 initialization"
            )

    def _require_prior_web_owner_released(self) -> None:
        try:
            nodes = subprocess.run(
                ["ros2", "node", "list"], capture_output=True, text=True, timeout=5.0, check=False
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f"cannot verify completed control-rights takeover: {exc}") from exc
        if nodes.returncode != 0:
            raise RuntimeError("cannot list ROS nodes after control-rights takeover")
        for node in nodes.stdout.splitlines():
            node = node.strip()
            if not node.startswith(self.takeover_owner_prefix):
                continue
            try:
                info = subprocess.run(
                    ["ros2", "node", "info", node], capture_output=True, text=True, timeout=5.0, check=False
                )
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError(f"timed out verifying prior control owner {node}") from exc
            service_servers = ""
            if info.returncode == 0 and "Service Servers:" in info.stdout:
                service_servers = info.stdout.split("Service Servers:", 1)[1].split("Service Clients:", 1)[0]
            if "/astribot/control_rights:" in service_servers:
                raise RuntimeError(f"prior Web control owner {node} did not release control rights after takeover")

    def _ordered_names(self) -> list[str]:
        return [self._names["left"], self._names["right"]]

    def _resolve_frame(self, frame: str) -> str:
        if frame == "chassis":
            return self._chassis_frame
        if frame != self._chassis_frame:
            raise ValueError(f"unsupported frame: {frame}")
        return frame

    def _parse_pose_response(self, values, label: str) -> dict[str, ArmTarget]:
        if not isinstance(values, (list, tuple)) or len(values) != 2:
            raise RuntimeError(f"Astribot {label} pose response must contain left and right poses")
        try:
            return {side: ArmTarget.from_pose_list(values[index]) for index, side in enumerate(("left", "right"))}
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"Astribot {label} pose response is invalid: {exc}") from exc

    def _validated_targets(self, targets: Mapping[str, ArmTarget], allow_subset: bool = False) -> dict[str, ArmTarget]:
        if not isinstance(targets, Mapping):
            raise ValueError("targets must be a mapping")
        expected = {"left", "right"}
        keys = set(targets)
        if not keys.issubset(expected) or (not allow_subset and keys != expected):
            requirement = "left and/or right" if allow_subset else "both left and right"
            raise ValueError(f"targets must contain {requirement}")
        return {side: self._copy_target(targets[side]) for side in targets}

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError("adapter is not initialized")

    @staticmethod
    def _copy_target(target: ArmTarget) -> ArmTarget:
        if not isinstance(target, ArmTarget):
            raise ValueError("each target must be an ArmTarget")
        return ArmTarget(target.position_array().tolist(), target.orientation_array().tolist())

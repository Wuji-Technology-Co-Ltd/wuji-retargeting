from __future__ import annotations

import argparse
import json
import socket
from typing import Sequence

from stardust_wuji_quest3_pc_retargeting.hand_control.ros2_bridge_core import Ros2BridgeCore
from stardust_wuji_quest3_pc_retargeting.hand_control.wujihand_ros2_publisher import (
    JOINT_NAMES_NO_PREFIX,
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive supervisor dual-hand frames and publish ROS2 recording topics."
    )
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=9011)
    parser.add_argument("--poll-rate-hz", type=float, default=200.0)
    parser.add_argument("--command-timeout-sec", type=float, default=0.25)
    parser.add_argument(
        "--publish-driver-commands",
        action="store_true",
        help=(
            "also publish /left_hand and /right_hand joint_commands; "
            "disabled by default so this bridge cannot move real hands"
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not 1 <= int(args.listen_port) <= 65535:
        raise SystemExit("listen port must be in [1, 65535]")
    if float(args.poll_rate_hz) <= 0.0:
        raise SystemExit("poll rate must be positive")

    import rclpy
    from geometry_msgs.msg import Pose, PoseArray
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    class WujiHandRos2BridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("quest3_wujihand_ros2_bridge")
            self.core = Ros2BridgeCore(args.command_timeout_sec)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setblocking(False)
            self.socket.bind((str(args.listen_host), int(args.listen_port)))
            self.raw_pubs = {
                side: self.create_publisher(
                    JointState, f"/teleop/hand/{side}/target_raw", sensor_qos
                )
                for side in ("left", "right")
            }
            self.safe_pubs = {
                side: self.create_publisher(
                    JointState, f"/teleop/hand/{side}/target_safe", sensor_qos
                )
                for side in ("left", "right")
            }
            self.keypoint_pubs = {
                side: self.create_publisher(
                    PoseArray, f"/teleop/quest3/{side}/keypoints", sensor_qos
                )
                for side in ("left", "right")
            }
            self.driver_pubs = (
                {
                    side: self.create_publisher(
                        JointState, f"/{side}_hand/joint_commands", sensor_qos
                    )
                    for side in ("left", "right")
                }
                if args.publish_driver_commands
                else {}
            )
            self.state_pub = self.create_publisher(String, "/teleop/hand/bridge_state", 10)
            self.last_stale_reported = True
            self.timer = self.create_timer(1.0 / float(args.poll_rate_hz), self.poll)
            self.get_logger().info(
                f"Listening for hand frames on udp://{args.listen_host}:{args.listen_port}; "
                f"driver_commands={'ENABLED' if args.publish_driver_commands else 'DISABLED'}"
            )

        def poll(self) -> None:
            for _ in range(100):
                try:
                    payload, _peer = self.socket.recvfrom(65_535)
                except BlockingIOError:
                    break
                try:
                    frame = self.core.ingest(payload)
                except Exception as exc:
                    self.get_logger().warning(f"Rejected hand bridge frame: {exc}")
                    continue
                if frame is not None:
                    self.publish_frame(frame)
                    self.last_stale_reported = False
            if self.core.stale() and not self.last_stale_reported:
                self.publish_state(None, stale=True)
                self.last_stale_reported = True

        def publish_frame(self, frame) -> None:
            stamp = self.get_clock().now().to_msg()
            for side in ("left", "right"):
                hand = frame.hands[side]
                names = [f"{side}_{name}" for name in JOINT_NAMES_NO_PREFIX]

                raw = JointState()
                raw.header.stamp = stamp
                raw.header.frame_id = f"wuji_hand_{side}"
                raw.name = names
                raw.position = list(hand.raw_qpos)
                self.raw_pubs[side].publish(raw)

                safe = JointState()
                safe.header.stamp = stamp
                safe.header.frame_id = f"wuji_hand_{side}"
                safe.name = names
                safe.position = list(hand.safe_qpos)
                self.safe_pubs[side].publish(safe)

                points = PoseArray()
                points.header.stamp = stamp
                points.header.frame_id = f"quest3_{side}_wrist"
                for xyz in hand.mp21:
                    pose = Pose()
                    pose.position.x = float(xyz[0])
                    pose.position.y = float(xyz[1])
                    pose.position.z = float(xyz[2])
                    pose.orientation.w = 1.0
                    points.poses.append(pose)
                self.keypoint_pubs[side].publish(points)

                if hand.enabled and side in self.driver_pubs:
                    self.driver_pubs[side].publish(safe)
            self.publish_state(frame, stale=False)

        def publish_state(self, frame, *, stale: bool) -> None:
            message = String()
            message.data = json.dumps(
                {
                    "schema": "quest3_wujihand_bridge_state.v1",
                    "stale": bool(stale),
                    "driver_commands_enabled": bool(args.publish_driver_commands),
                    "accepted_frames": self.core.stats.accepted_frames,
                    "rejected_frames": self.core.stats.rejected_frames,
                    "duplicate_frames": self.core.stats.duplicate_frames,
                    "out_of_order_frames": self.core.stats.out_of_order_frames,
                    "seq": None if frame is None else frame.seq,
                    "client_time_sec": None if frame is None else frame.client_time_sec,
                    "control_pc_receive_time_ns": (
                        None if frame is None else frame.receive_time_ns
                    ),
                    "xr_session_id": "" if frame is None else frame.xr_session_id,
                    "teleop_state": "STALE" if frame is None else frame.teleop_state,
                    "command_enabled": (
                        {"left": False, "right": False}
                        if frame is None
                        else {
                            side: bool(frame.hands[side].enabled)
                            for side in ("left", "right")
                        }
                    ),
                },
                separators=(",", ":"),
                sort_keys=True,
            )
            self.state_pub.publish(message)

        def destroy_node(self):
            self.socket.close()
            return super().destroy_node()

    rclpy.init(args=None)
    node = WujiHandRos2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

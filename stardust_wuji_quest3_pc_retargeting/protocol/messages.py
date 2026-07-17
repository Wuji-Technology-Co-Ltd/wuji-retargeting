from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

SCHEMA = "quest3_web_teleop.v1"


@dataclass
class PoseFrame:
    valid: bool
    position: list[float]
    orientation_xyzw: list[float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HandFrame:
    valid: bool
    joint_names: list[str]
    positions: list[list[float]]
    orientations_xyzw: list[list[float]]
    confidence: float | None = None

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        if data["confidence"] is None:
            data.pop("confidence")
        return data


@dataclass
class SessionFrame:
    active: bool
    visibility: str = "visible"
    reference_space: str = "local-floor"
    reference_space_revision: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TrackingFrame:
    seq: int
    client_time_sec: float
    hmd: PoseFrame
    hands: dict[str, HandFrame]
    arm_wrists: dict[str, PoseFrame]
    session: SessionFrame
    xr_session_id: str = ""
    schema: str = SCHEMA
    type: str = "tracking_frame"

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": self.schema,
            "type": self.type,
            "seq": self.seq,
            "client_time_sec": self.client_time_sec,
            "xr_session_id": self.xr_session_id,
            "hmd": self.hmd.to_dict(),
            "hands": {side: hand.to_dict() for side, hand in self.hands.items()},
            "arm_wrists": {side: wrist.to_dict() for side, wrist in self.arm_wrists.items()},
            "session": self.session.to_dict(),
        }

from __future__ import annotations

import math
from typing import Any, Iterable

from .messages import SCHEMA, HandFrame, PoseFrame, SessionFrame, TrackingFrame


class ProtocolError(ValueError):
    """Raised when a WebXR teleop message violates the wire contract."""


def _require_mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProtocolError(f"{name} must be an object")
    return value


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ProtocolError(f"{name} must be finite")
    return number


def _finite_vector(value: Any, size: int, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise ProtocolError(f"{name} must be a {size}-element list")
    return [_finite_float(item, f"{name}[{idx}]") for idx, item in enumerate(value)]


def _finite_matrix(value: Any, width: int, name: str) -> list[list[float]]:
    if not isinstance(value, list):
        raise ProtocolError(f"{name} must be a list")
    return [_finite_vector(row, width, f"{name}[{idx}]") for idx, row in enumerate(value)]


def _string_list(value: Any, name: str) -> list[str]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ProtocolError(f"{name} must be a list of strings")
    return list(value)


def _require_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ProtocolError(f"{name} must be a boolean")
    return value


def validate_pose(payload: Any, name: str) -> PoseFrame:
    data = _require_mapping(payload, name)
    return PoseFrame(
        valid=_require_bool(data.get("valid"), f"{name}.valid"),
        position=_finite_vector(data.get("position"), 3, f"{name}.position"),
        orientation_xyzw=_finite_vector(data.get("orientation_xyzw"), 4, f"{name}.orientation_xyzw"),
    )


def validate_hand(payload: Any, side: str) -> HandFrame:
    data = _require_mapping(payload, f"hands.{side}")
    valid = _require_bool(data.get("valid"), f"hands.{side}.valid")
    names = _string_list(data.get("joint_names"), f"hands.{side}.joint_names")
    positions = _finite_matrix(data.get("positions"), 3, f"hands.{side}.positions")
    orientations = _finite_matrix(
        data.get("orientations_xyzw"),
        4,
        f"hands.{side}.orientations_xyzw",
    )
    if len(names) != len(positions) or len(names) != len(orientations):
        raise ProtocolError(f"hands.{side} joint_names, positions, and orientations must have equal length")
    confidence = data.get("confidence")
    return HandFrame(
        valid=valid,
        joint_names=names,
        positions=positions,
        orientations_xyzw=orientations,
        confidence=None if confidence is None else _finite_float(confidence, f"hands.{side}.confidence"),
    )


def validate_tracking_frame(payload: Any) -> TrackingFrame:
    data = _require_mapping(payload, "message")
    if data.get("schema") != SCHEMA:
        raise ProtocolError(f"schema must be {SCHEMA}")
    if data.get("type") != "tracking_frame":
        raise ProtocolError("type must be tracking_frame")
    seq = data.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        raise ProtocolError("seq must be a non-negative integer")
    hands_data = _require_mapping(data.get("hands"), "hands")
    hands = {side: validate_hand(hands_data.get(side), side) for side in ("left", "right")}
    session_data = _require_mapping(data.get("session"), "session")
    visibility = session_data.get("visibility", "visible")
    reference_space = session_data.get("reference_space", "local-floor")
    if not isinstance(visibility, str) or not isinstance(reference_space, str):
        raise ProtocolError("session visibility and reference_space must be strings")
    revision = session_data.get("reference_space_revision")
    if revision is not None and (isinstance(revision, bool) or not isinstance(revision, int) or revision < 0):
        raise ProtocolError("session.reference_space_revision must be a non-negative integer")
    xr_session_id = data.get("xr_session_id", "")
    if not isinstance(xr_session_id, str):
        raise ProtocolError("xr_session_id must be a string")
    return TrackingFrame(
        seq=seq,
        client_time_sec=_finite_float(data.get("client_time_sec"), "client_time_sec"),
        xr_session_id=xr_session_id,
        hmd=validate_pose(data.get("hmd"), "hmd"),
        hands=hands,
        session=SessionFrame(
            active=_require_bool(session_data.get("active"), "session.active"),
            visibility=visibility,
            reference_space=reference_space,
            reference_space_revision=revision,
        ),
    )

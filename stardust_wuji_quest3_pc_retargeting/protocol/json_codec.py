from __future__ import annotations

import json
from typing import Any

from .messages import TrackingFrame
from .validation import ProtocolError, validate_tracking_frame


def decode_message(raw: str | bytes) -> TrackingFrame:
    try:
        payload: Any = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"invalid JSON: {exc}") from exc
    return validate_tracking_frame(payload)


def encode_message(message: TrackingFrame | dict[str, Any]) -> str:
    if isinstance(message, TrackingFrame):
        payload = message.to_dict()
    elif isinstance(message, dict):
        payload = message
    else:
        raise TypeError(f"unsupported message type: {type(message)!r}")
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False)

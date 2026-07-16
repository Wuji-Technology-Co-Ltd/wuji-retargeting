from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class RelayCounters:
    quest_to_control_messages: int = 0
    quest_to_control_tracking_frames: int = 0
    control_to_quest_messages: int = 0
    errors: list[str] = field(default_factory=list)

    @property
    def quest_to_control(self) -> int:
        return self.quest_to_control_messages

    @property
    def control_to_quest(self) -> int:
        return self.control_to_quest_messages


def _decode_payload(message: str | bytes) -> dict[str, Any] | None:
    try:
        payload = json.loads(message)
    except (TypeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _should_log(count: int) -> bool:
    return count <= 5 or count % 60 == 0


def _format_debug_value(value: Any) -> str:
    if isinstance(value, (dict, list)):
        return json.dumps(value, separators=(",", ":"), ensure_ascii=False)
    return str(value)


def _emit_xr_debug(payload: dict[str, Any], emit: Callable[[str], None]) -> None:
    stage = payload.get("stage", "unknown")
    fields = []
    for key in (
        "seq",
        "frame_index",
        "reference_space",
        "viewer_pose",
        "input_sources",
        "left_valid",
        "right_valid",
        "error",
    ):
        if key in payload:
            fields.append(f"{key}={_format_debug_value(payload[key])}")
    suffix = f" {' '.join(fields)}" if fields else ""
    emit(f"[relay][xr_debug] stage={stage}{suffix}")


def record_quest_message(
    message: str | bytes,
    counters: RelayCounters,
    emit: Callable[[str], None] = print,
) -> bool:
    counters.quest_to_control_messages += 1
    payload = _decode_payload(message)
    message_type = payload.get("type") if payload else "raw"

    if message_type == "xr_debug" and payload is not None:
        _emit_xr_debug(payload, emit)
        return False

    if message_type == "tracking_frame":
        counters.quest_to_control_tracking_frames += 1
        if _should_log(counters.quest_to_control_tracking_frames):
            emit(
                "[relay] quest->control "
                f"tracking_frames={counters.quest_to_control_tracking_frames} "
                f"messages={counters.quest_to_control_messages}"
            )
    elif _should_log(counters.quest_to_control_messages):
        emit(
            "[relay] quest->control "
            f"messages={counters.quest_to_control_messages} "
            f"type={message_type} "
            f"tracking_frames={counters.quest_to_control_tracking_frames}"
        )
    return True


def record_control_message(
    counters: RelayCounters,
    emit: Callable[[str], None] = print,
) -> None:
    counters.control_to_quest_messages += 1
    if _should_log(counters.control_to_quest_messages):
        emit(f"[relay] control->quest messages={counters.control_to_quest_messages}")

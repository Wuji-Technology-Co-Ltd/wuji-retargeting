from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GatewayStatus:
    quest_connected: bool = False
    control_connected: bool = False
    forwarded_from_quest: int = 0
    forwarded_from_control: int = 0
    last_error: str = ""

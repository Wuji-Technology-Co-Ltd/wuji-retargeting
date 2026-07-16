from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class DryRunRobot:
    hand_commands: list[object] = field(default_factory=list)
    arm_commands: list[object] = field(default_factory=list)

    def send_hand(self, side: str, command) -> None:
        self.hand_commands.append((side, command))

    def send_arm(self, side: str, command) -> None:
        self.arm_commands.append((side, command))

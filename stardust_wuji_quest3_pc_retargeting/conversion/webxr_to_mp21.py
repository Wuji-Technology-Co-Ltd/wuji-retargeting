from __future__ import annotations

from pathlib import Path
from typing import Mapping

import numpy as np
import yaml

from stardust_wuji_quest3_pc_retargeting.protocol.messages import HandFrame
from stardust_wuji_quest3_pc_retargeting.protocol.validation import ProtocolError

from .hand_joint_names import DEFAULT_WEBXR_TO_MP21, LEGACY_QUEST26_TO_WEBXR, MP21_NAMES


class WebXRToMP21Converter:
    def __init__(
        self,
        mapping: Mapping[str, str] | None = None,
        scale: float = 1.0,
        wrist_relative: bool = True,
        axis_transform: np.ndarray | None = None,
    ):
        raw_mapping = dict(mapping or DEFAULT_WEBXR_TO_MP21)
        self.mapping = {mp21: LEGACY_QUEST26_TO_WEBXR.get(webxr, webxr) for mp21, webxr in raw_mapping.items()}
        self.scale = float(scale)
        self.wrist_relative = bool(wrist_relative)
        self.axis_transform = np.asarray(axis_transform if axis_transform is not None else np.eye(3), dtype=float)
        if self.axis_transform.shape != (3, 3):
            raise ValueError("axis_transform must be 3x3")

    @classmethod
    def from_yaml(cls, path: str | Path) -> "WebXRToMP21Converter":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls(
            mapping=data.get("mapping"),
            scale=data.get("scale", 1.0),
            wrist_relative=data.get("wrist_relative", True),
            axis_transform=np.asarray(data.get("axis_transform", np.eye(3)), dtype=float),
        )

    def convert(self, hand: HandFrame) -> np.ndarray:
        if not hand.valid:
            return np.zeros((21, 3), dtype=float)
        by_name = {name: np.asarray(pos, dtype=float) for name, pos in zip(hand.joint_names, hand.positions)}
        missing = [self.mapping[name] for name in MP21_NAMES if self.mapping[name] not in by_name]
        if missing:
            raise ProtocolError(f"missing WebXR hand joints: {missing}")
        points = np.vstack([by_name[self.mapping[name]] for name in MP21_NAMES]).astype(float)
        if not np.isfinite(points).all():
            raise ProtocolError("hand positions must be finite")
        if self.wrist_relative:
            points = points - points[0]
        points = (points @ self.axis_transform.T) * self.scale
        return points

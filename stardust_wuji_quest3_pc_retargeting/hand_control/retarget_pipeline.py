from __future__ import annotations

import numpy as np


class RetargetPipeline:
    """Optional Wuji Retargeter wrapper with a deterministic dry-run fallback."""

    def __init__(self, config_path: str | None = None, dry_run: bool = True):
        self.config_path = config_path
        self.dry_run = dry_run
        self._retargeter = None
        if not dry_run and config_path:
            from wuji_retargeting import Retargeter

            self._retargeter = Retargeter.from_yaml(config_path)

    def retarget(self, mp21_points) -> np.ndarray:
        points = np.asarray(mp21_points, dtype=float)
        if points.shape != (21, 3):
            raise ValueError(f"mp21_points must be (21, 3), got {points.shape}")
        if self._retargeter is not None:
            return np.asarray(self._retargeter.retarget(points), dtype=float).reshape(20)
        distances = np.linalg.norm(points[1:] - points[0], axis=1)
        if distances.size < 20:
            distances = np.pad(distances, (0, 20 - distances.size))
        return np.clip(distances[:20] * 10.0, 0.0, 1.0)

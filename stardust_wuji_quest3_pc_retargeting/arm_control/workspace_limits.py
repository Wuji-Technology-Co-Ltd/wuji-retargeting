from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class WorkspaceLimits:
    xyz_min: list[float]
    xyz_max: list[float]

    def clip(self, position) -> np.ndarray:
        pos = np.asarray(position, dtype=float)
        if pos.shape != (3,) or not np.isfinite(pos).all():
            raise ValueError("position must be 3 finite values")
        return np.clip(pos, np.asarray(self.xyz_min, dtype=float), np.asarray(self.xyz_max, dtype=float))

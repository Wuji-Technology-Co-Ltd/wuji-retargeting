"""Base classes and utilities for hand retargeting optimizers."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import nlopt
import numpy as np
import yaml

from ..robot import RobotWrapper


@dataclass
class TimingStats:
    """Timing statistics for optimizer performance analysis."""
    preprocess_ms: float = 0.0
    fk_ms: float = 0.0
    jacobian_ms: float = 0.0
    gradient_ms: float = 0.0
    nlopt_ms: float = 0.0
    total_ms: float = 0.0
    call_count: int = 0
    iter_counts: List[int] = field(default_factory=list)
    # Per-frame iteration losses: list of lists, each inner list is losses per iteration
    iter_losses: List[List[float]] = field(default_factory=list)
    # Current frame's iteration losses (temporary storage during optimization)
    _current_iter_losses: List[float] = field(default_factory=list)

    def reset(self):
        """Reset all timing statistics."""
        self.preprocess_ms = 0.0
        self.fk_ms = 0.0
        self.jacobian_ms = 0.0
        self.gradient_ms = 0.0
        self.nlopt_ms = 0.0
        self.total_ms = 0.0
        self.call_count = 0
        self.iter_counts = []
        self.iter_losses = []
        self._current_iter_losses = []

    def start_frame(self):
        """Start recording for a new frame."""
        self._current_iter_losses = []

    def record_iter_loss(self, loss: float):
        """Record loss for current iteration."""
        self._current_iter_losses.append(loss)

    def end_frame(self, num_evals: int):
        """End recording for current frame."""
        self.iter_counts.append(num_evals)
        self.iter_losses.append(self._current_iter_losses.copy())
        self._current_iter_losses = []

    def get_last_iter_losses(self) -> List[float]:
        """Get iteration losses for the last frame."""
        if self.iter_losses:
            return self.iter_losses[-1]
        return []

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary."""
        return {
            'preprocess_ms': self.preprocess_ms,
            'fk_ms': self.fk_ms,
            'jacobian_ms': self.jacobian_ms,
            'gradient_ms': self.gradient_ms,
            'nlopt_ms': self.nlopt_ms,
            'total_ms': self.total_ms,
            'call_count': self.call_count,
        }

    def get_avg(self) -> Dict[str, float]:
        """Get average timing per call."""
        if self.call_count == 0:
            return self.to_dict()
        return {
            'preprocess_ms': self.preprocess_ms / self.call_count,
            'fk_ms': self.fk_ms / self.call_count,
            'jacobian_ms': self.jacobian_ms / self.call_count,
            'gradient_ms': self.gradient_ms / self.call_count,
            'nlopt_ms': self.nlopt_ms / self.call_count,
            'total_ms': self.total_ms / self.call_count,
            'call_count': self.call_count,
        }

    def get_iter_stats(self) -> Dict[str, float]:
        """Get iteration count statistics."""
        if not self.iter_counts:
            return {}
        arr = np.array(self.iter_counts)
        return {
            'min': int(np.min(arr)),
            'max': int(np.max(arr)),
            'mean': float(np.mean(arr)),
            'std': float(np.std(arr)),
            'median': float(np.median(arr)),
            'p90': float(np.percentile(arr, 90)),
            'p99': float(np.percentile(arr, 99)),
        }


# Package root for URDF path resolution
_THIS_FILE = Path(__file__).resolve()
_PACKAGE_ROOT = _THIS_FILE.parent.parent

# Unit conversion: internal computations use cm
M_TO_CM = 100.0
CM_TO_M = 0.01


def huber_loss_np(x: np.ndarray, delta: float = 2.0) -> np.ndarray:
    """Huber loss function (smooth L1 loss)."""
    abs_x = np.abs(x)
    return np.where(
        abs_x <= delta,
        0.5 * x ** 2,
        delta * (abs_x - 0.5 * delta)
    )


def huber_loss_grad_np(x: np.ndarray, delta: float = 2.0) -> np.ndarray:
    """Gradient of Huber loss w.r.t. x (numpy version)."""
    abs_x = np.abs(x)
    return np.where(abs_x <= delta, x, delta * np.sign(x))


class LPFilter:
    """Low-pass filter for smoothing joint positions."""

    def __init__(self, alpha: float):
        """Initialize filter.

        Args:
            alpha: Filter coefficient (0 < alpha <= 1).
                   Smaller = smoother but more latency.
        """
        self.alpha = alpha
        self.y = None
        self.is_init = False

    def next(self, x: np.ndarray) -> np.ndarray:
        """Apply filter to new value."""
        if not self.is_init:
            self.y = x.copy()
            self.is_init = True
            return self.y.copy()
        self.y = self.y + self.alpha * (x - self.y)
        return self.y.copy()

    def reset(self):
        """Reset filter state."""
        self.y = None
        self.is_init = False


class BaseOptimizer(ABC):
    """Base class for hand retargeting optimizers.

    All parameters are read from configuration dict (loaded from YAML).
    """

    # MediaPipe keypoint indices
    MP_ORIGIN_IDX = 0  # Wrist
    MP_TIP_INDICES = [4, 8, 12, 16, 20]  # Fingertips
    MP_PIP_INDICES = [2, 6, 10, 14, 18]  # PIP joints (thumb uses MCP=2)
    MP_DIP_INDICES = [3, 7, 11, 15, 19]  # DIP joints

    def __init__(self, config: dict):
        """Initialize optimizer from configuration dict.

        Args:
            config: Configuration dict (typically loaded from YAML)
        """
        self.config = config

        # Extract optimizer config
        opt_config = config.get('optimizer', {})
        self.hand_side = opt_config.get('hand_side', 'right').lower()
        if self.hand_side not in ['right', 'left']:
            raise ValueError(f"hand_side must be 'right' or 'left', got {self.hand_side}")

        # Extract retarget config
        retarget_config = config.get('retarget', {})
        self.huber_delta = retarget_config.get('huber_delta', 2.0)
        self.norm_delta = retarget_config.get('norm_delta', 0.04)

        # Load URDF
        urdf_path = str((_PACKAGE_ROOT / f"wuji_hand_description/urdf/{self.hand_side}.urdf").resolve())
        self.robot = RobotWrapper(urdf_path, hand_side=self.hand_side)
        self.num_joints = self.robot.model.nq

        # Setup NLopt optimizer
        self.opt = nlopt.opt(nlopt.LD_SLSQP, self.num_joints)
        self.opt.set_maxeval(50)      # Reduced from 1000 for faster convergence with warm-start
        self.opt.set_ftol_abs(1e-4)   # Relaxed from 1e-6 for faster convergence
        self.opt.set_lower_bounds(self.robot.joint_limits[:, 0].tolist())
        self.opt.set_upper_bounds(self.robot.joint_limits[:, 1].tolist())

        # Link names
        self.origin_link_name = "palm_link"
        self.task_link_names = [f"finger{i}_tip_link" for i in range(1, 6)]
        self.link3_names = [f"finger{i}_link3" for i in range(1, 6)]
        self.link4_names = [f"finger{i}_link4" for i in range(1, 6)]

        # Build link indices
        self._build_link_indices()

        # Store last solution for warm start
        self.last_qpos = None

    def _build_link_indices(self):
        """Build link indices for FK computation."""
        # Collect all needed link names
        all_link_names = (
            [self.origin_link_name] +
            self.task_link_names +
            self.link3_names +
            self.link4_names
        )
        self.computed_link_names = list(dict.fromkeys(all_link_names))
        self.computed_link_indices = [
            self.robot.get_link_index(name) for name in self.computed_link_names
        ]

        # Build index mappings
        self.origin_indices = [
            self.computed_link_names.index(self.origin_link_name)
            for _ in range(5)
        ]
        self.task_indices = [
            self.computed_link_names.index(name) for name in self.task_link_names
        ]
        self.link3_indices = [
            self.computed_link_names.index(name) for name in self.link3_names
        ]
        self.link4_indices = [
            self.computed_link_names.index(name) for name in self.link4_names
        ]

    @classmethod
    def from_yaml(cls, yaml_path: str, hand_side: str = None) -> "BaseOptimizer":
        """Create optimizer from YAML configuration file.

        Args:
            yaml_path: Path to YAML configuration file
            hand_side: Optional hand side override ('left' or 'right')

        Returns:
            Optimizer instance
        """
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        # Override hand_side if provided
        if hand_side is not None:
            if 'optimizer' not in config:
                config['optimizer'] = {}
            config['optimizer']['hand_side'] = hand_side

        return cls.from_config(config)

    @classmethod
    def from_config(cls, config: dict) -> "BaseOptimizer":
        """Create optimizer from configuration dict.

        Args:
            config: Configuration dict

        Returns:
            Optimizer instance
        """
        from .adaptive_analytical import AdaptiveOptimizerAnalytical
        from .vector import VectorOptimizer

        registry = {
            'AdaptiveOptimizerAnalytical': AdaptiveOptimizerAnalytical,
            'VectorOptimizer': VectorOptimizer,
        }
        opt_type = config.get('optimizer', {}).get('type', 'AdaptiveOptimizerAnalytical')

        if opt_type not in registry:
            raise ValueError(
                f"Unknown optimizer type: {opt_type}. "
                f"Supported: {sorted(registry)}"
            )
        return registry[opt_type](config)

    @abstractmethod
    def solve(
        self,
        mediapipe_keypoints: np.ndarray,
        last_qpos: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Solve for joint angles.

        Args:
            mediapipe_keypoints: (21, 3) MediaPipe keypoints in wrist frame
            last_qpos: Initial guess for optimization (warm start)

        Returns:
            qpos: (num_joints,) joint angles
        """
        pass

    @abstractmethod
    def compute_cost(
        self,
        qpos: np.ndarray,
        mediapipe_keypoints: np.ndarray,
    ) -> float:
        """Compute cost for given joint angles.

        Args:
            qpos: Joint angles
            mediapipe_keypoints: (21, 3) MediaPipe keypoints

        Returns:
            cost: Total loss value
        """
        pass

    # =========================================================================
    # Common helper methods (shared by subclasses)
    # =========================================================================

    def _get_init_qpos(self, last_qpos: Optional[np.ndarray]) -> np.ndarray:
        """Get initial qpos for optimization (clipped to joint limits).

        Args:
            last_qpos: Optional last qpos from caller

        Returns:
            Initial qpos for optimization
        """
        if last_qpos is not None:
            init_qpos = np.asarray(last_qpos, dtype=np.float64)
        elif self.last_qpos is not None:
            init_qpos = self.last_qpos
        else:
            init_qpos = self.robot.joint_limits.mean(axis=1)

        return np.clip(
            init_qpos,
            self.robot.joint_limits[:, 0],
            self.robot.joint_limits[:, 1]
        )

    def _get_reg_qpos(self, last_qpos: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """Get regularization qpos for norm_delta term.

        Args:
            last_qpos: Optional last qpos from caller

        Returns:
            Regularization qpos or None
        """
        if last_qpos is not None:
            return np.asarray(last_qpos, dtype=np.float64)
        elif self.last_qpos is not None:
            return self.last_qpos
        return None

    def _run_optimization(self, objective_fn, init_qpos: np.ndarray) -> np.ndarray:
        """Run NLopt optimization and update last_qpos.

        Args:
            objective_fn: NLopt objective function
            init_qpos: Initial qpos

        Returns:
            Optimized qpos
        """
        self.opt.set_min_objective(objective_fn)
        try:
            qpos = self.opt.optimize(init_qpos.tolist())
            qpos = np.array(qpos, dtype=np.float32)
        except RuntimeError as e:
            print(f"[{self.__class__.__name__}] Optimization failed: {e}")
            qpos = np.array(init_qpos, dtype=np.float32)
        self.last_qpos = qpos.astype(np.float64)
        return qpos

    def _compute_tip_vectors(self, keypoints: np.ndarray, scaling: float = 1.0) -> np.ndarray:
        """Compute wrist->tip vectors.

        Args:
            keypoints: (21, 3) MediaPipe keypoints in meters
            scaling: Global scaling factor

        Returns:
            vectors: (5, 3) tip vectors in cm
        """
        wrist = keypoints[self.MP_ORIGIN_IDX]
        vectors = np.array([
            keypoints[idx] - wrist for idx in self.MP_TIP_INDICES
        ]) * scaling * M_TO_CM
        return vectors.astype(np.float64)

    def _compute_tip_dirs(self, keypoints: np.ndarray) -> np.ndarray:
        """Compute DIP->tip direction vectors (normalized).

        Args:
            keypoints: (21, 3) MediaPipe keypoints

        Returns:
            tip_dirs: (5, 3) normalized direction vectors
        """
        tip_dirs = []
        for dip_idx, tip_idx in zip(self.MP_DIP_INDICES, self.MP_TIP_INDICES):
            dir_vec = keypoints[tip_idx] - keypoints[dip_idx]
            norm = np.linalg.norm(dir_vec)
            tip_dirs.append(dir_vec / (norm + 1e-8))
        return np.array(tip_dirs, dtype=np.float64)

    def _compute_full_hand_vectors(self, keypoints: np.ndarray, scaling: np.ndarray) -> np.ndarray:
        """Compute full hand vectors (wrist->PIP, wrist->DIP, wrist->TIP).

        Args:
            keypoints: (21, 3) MediaPipe keypoints in meters
            scaling: (5, 3) scaling factors for each finger and segment

        Returns:
            vectors: (15, 3) vectors in cm [PIP*5, DIP*5, TIP*5]
        """
        wrist = keypoints[self.MP_ORIGIN_IDX]

        # wrist -> PIP (5 vectors)
        pip_vectors = np.array([
            keypoints[idx] - wrist for idx in self.MP_PIP_INDICES
        ]) * scaling[:, 0:1]

        # wrist -> DIP (5 vectors)
        dip_vectors = np.array([
            keypoints[idx] - wrist for idx in self.MP_DIP_INDICES
        ]) * scaling[:, 1:2]

        # wrist -> TIP (5 vectors)
        tip_vectors = np.array([
            keypoints[idx] - wrist for idx in self.MP_TIP_INDICES
        ]) * scaling[:, 2:3]

        # Concatenate and convert to cm
        vectors = np.vstack([pip_vectors, dip_vectors, tip_vectors]) * M_TO_CM
        return vectors.astype(np.float64)

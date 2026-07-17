"""Interactive MuJoCo viewer for hand retargeting parameter tuning.

Core component of the tuning visualization tool. Provides:
- Three-layer skeleton display (input/scaled/FK)
- YAML config hot-reload for real-time parameter tuning
- Frame navigation (pause, step, seek)
- HUD overlay with parameter values and tuning guidance

Usage:
    viewer = TuningViewer(
        hand_side="left",
        retarget_config_path="config/adaptive_analytical_avp.yaml",
    )
    viewer.play_recording(data)
"""

from __future__ import annotations

import pickle
import signal
import time
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np
import yaml

from ..retarget import Retargeter
from ..mediapipe import apply_mediapipe_transformations
from .skeleton_drawer import SkeletonDrawer
from .config_watcher import ConfigWatcher
from .param_map import get_affected_fingers


# Finger indices for computing scaled keypoints
_FINGER_INDICES = [
    [1, 2, 3, 4],       # thumb
    [5, 6, 7, 8],       # index
    [9, 10, 11, 12],    # middle
    [13, 14, 15, 16],   # ring
    [17, 18, 19, 20],   # pinky
]
_JOINT_TO_SEGMENT = [0, 1, 2, 3]


def _compute_scaled_mediapipe(mediapipe_kp: np.ndarray, optimizer) -> np.ndarray:
    """Compute scaled MediaPipe keypoints based on optimizer segment_scaling.

    Args:
        mediapipe_kp: (21, 3) keypoints in wrist frame
        optimizer: Optimizer instance with segment_scaling_full attribute

    Returns:
        Scaled keypoints (21, 3)
    """
    wrist_kp = mediapipe_kp[0]
    scaled_kp = mediapipe_kp.copy()

    scaling_matrix = getattr(optimizer, "segment_scaling_full", None)
    if scaling_matrix is None:
        scaling_matrix = getattr(optimizer, "segment_scaling", None)
    if scaling_matrix is None:
        return scaled_kp

    for finger_idx, joints in enumerate(_FINGER_INDICES):
        for j, joint_idx in enumerate(joints):
            segment_idx = _JOINT_TO_SEGMENT[j]
            if scaling_matrix.shape[1] > segment_idx:
                scale = scaling_matrix[finger_idx, segment_idx]
            else:
                scale = 1.0
            vec_from_wrist = mediapipe_kp[joint_idx] - wrist_kp
            scaled_kp[joint_idx] = wrist_kp + vec_from_wrist * scale

    return scaled_kp


class TuningViewer:
    """Interactive MuJoCo viewer for parameter tuning.

    Displays three skeleton layers simultaneously and supports
    real-time config hot-reload for interactive parameter tuning.

    Args:
        hand_side: 'left' or 'right'
        retarget_config_path: Path to retarget YAML config
        viz_config_path: Optional path to visualization YAML config (tuning_viz.yaml)
        mujoco_model_dir: Optional override for MuJoCo model directory
    """

    def __init__(
        self,
        hand_side: str = "left",
        retarget_config_path: str = None,
        viz_config_path: str = None,
        mujoco_model_dir: str = None,
    ):
        self.hand_side = hand_side.lower()

        # Resolve retarget config path
        if retarget_config_path is None:
            retarget_config_path = str(
                Path(__file__).resolve().parents[2]
                / "example" / "config" / "adaptive_analytical_avp.yaml"
            )
        self.retarget_config_path = Path(retarget_config_path).resolve()

        # Load retarget config
        with open(self.retarget_config_path, "r") as f:
            self.retarget_config = yaml.safe_load(f)

        # Load viz config
        self.viz_config = {}
        if viz_config_path is not None:
            viz_path = Path(viz_config_path).resolve()
            if viz_path.exists():
                with open(viz_path, "r") as f:
                    self.viz_config = yaml.safe_load(f) or {}

        # Load MuJoCo model
        if mujoco_model_dir is None:
            mujoco_model_dir = (
                Path(__file__).resolve().parents[2]
                / "example" / "utils" / "mujoco-sim"
            )
        mjcf_path = Path(mujoco_model_dir) / "wuji_hand_description" / "mjcf" / f"{self.hand_side}.xml"
        if not mjcf_path.exists():
            raise FileNotFoundError(f"MuJoCo model not found: {mjcf_path}")

        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)

        # Make hand mesh semi-transparent
        mesh_alpha = self.viz_config.get("robot_mesh", {}).get("alpha", 0.3)
        for i in range(self.model.ngeom):
            if self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
                self.model.geom_rgba[i, 3] = mesh_alpha

        # Initialize retargeter
        self.retargeter = Retargeter(self.retarget_config.copy(), self.hand_side)

        # Initialize skeleton drawer
        skeleton_config = self.viz_config.get("skeleton", {})
        self.drawer = SkeletonDrawer(
            self.model, self.data, self.hand_side, skeleton_config
        )

        # Initialize config watcher
        self.config_watcher = ConfigWatcher(
            str(self.retarget_config_path),
            poll_interval=self.viz_config.get("hot_reload", {}).get("poll_interval", 0.5),
            verbose=True,
        )

        # HUD settings
        hud_config = self.viz_config.get("hud", {})
        self.show_params = hud_config.get("show_params", True)
        self.show_frame_info = hud_config.get("show_frame_info", True)
        self.show_cost = hud_config.get("show_cost", True)
        self.show_pinch_alpha = hud_config.get("show_pinch_alpha", True)

        # Highlight state
        self._highlight_timer = 0.0
        self._highlight_duration = self.viz_config.get("skeleton", {}).get(
            "highlight_duration", 1.0
        )

    def _reload_retargeter(self, new_config: dict, changes: list):
        """Reload retargeter with new config and highlight affected fingers."""
        self.retarget_config = new_config
        self.retargeter = Retargeter(new_config.copy(), self.hand_side)
        self.retargeter.reset_filter()

        # Highlight affected fingers
        affected_fingers = set()
        for param_path, _, _ in changes:
            lookup_key = param_path
            if param_path.startswith("retarget."):
                lookup_key = param_path[len("retarget."):]
            fingers = get_affected_fingers(lookup_key)
            affected_fingers.update(fingers)

        if affected_fingers:
            self.drawer.set_highlight_fingers(list(affected_fingers))
            self._highlight_timer = time.time()

    def _check_highlight_timeout(self):
        """Clear highlight after duration expires."""
        if self._highlight_timer > 0:
            if time.time() - self._highlight_timer > self._highlight_duration:
                self.drawer.clear_highlight()
                self._highlight_timer = 0.0

    def _set_camera(self, viewer):
        """Set viewer camera from config."""
        cam_config = self.viz_config.get("camera", {})
        viewer.cam.azimuth = cam_config.get("azimuth", 135)
        viewer.cam.elevation = cam_config.get("elevation", -20)
        viewer.cam.distance = cam_config.get("distance", 0.5)
        lookat = cam_config.get("lookat", [0, 0, 0.05])
        viewer.cam.lookat[:] = lookat

    def _process_frame(self, raw_keypoints: np.ndarray) -> dict:
        """Process a single frame through the retargeting pipeline.

        Args:
            raw_keypoints: (21, 3) raw MediaPipe keypoints

        Returns:
            Dict with qpos, mediapipe_kp, scaled_kp, cost, pinch_alphas
        """
        qpos, verbose = self.retargeter.retarget_verbose(raw_keypoints)

        mediapipe_kp = verbose["mediapipe_kp"]
        scaled_kp = _compute_scaled_mediapipe(mediapipe_kp, self.retargeter.optimizer)

        return {
            "qpos": qpos,
            "mediapipe_kp": mediapipe_kp,
            "scaled_kp": scaled_kp,
            "cost": verbose.get("cost", 0.0),
            "pinch_alphas": verbose.get("pinch_alphas"),
        }

    def play_recording(
        self,
        data_or_path,
        fps: float = 30.0,
        hand_key: str = None,
    ):
        """Play a recording with interactive tuning visualization.

        Args:
            data_or_path: List of frame dicts, or path to .pkl file
            fps: Target playback framerate
            hand_key: Key for hand data in frame dict (default: auto from hand_side)
        """
        # Load data
        if isinstance(data_or_path, (str, Path)):
            with open(data_or_path, "rb") as f:
                data = pickle.load(f)
        else:
            data = data_or_path

        if not data:
            raise ValueError("No data to play")

        if hand_key is None:
            hand_key = f"{self.hand_side}_fingers"

        total_frames = len(data)
        frame_time = 1.0 / fps

        # Playback state
        current_frame = 0
        paused = False
        running = True
        last_result = None

        def signal_handler(_sig, _frame):
            nonlocal running
            running = False

        old_handler = signal.signal(signal.SIGINT, signal_handler)

        print(f"Tuning Viewer started")
        print(f"  Config: {self.retarget_config_path.name}")
        print(f"  Hand: {self.hand_side}")
        print(f"  Frames: {total_frames}")
        print(f"  FPS: {fps}")
        print(f"")
        print(f"Controls:")
        print(f"  Edit {self.retarget_config_path.name} to tune parameters (auto-reload)")
        print(f"  Close viewer window to exit")
        print(f"{'=' * 50}")

        # Initialize model
        for i in range(self.model.nu):
            if self.model.actuator_ctrllimited[i]:
                ctrl_range = self.model.actuator_ctrlrange[i]
                self.data.ctrl[i] = (ctrl_range[0] + ctrl_range[1]) / 2
            else:
                self.data.ctrl[i] = 0.0
        for _ in range(100):
            mujoco.mj_step(self.model, self.data)

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self._set_camera(viewer)

            while viewer.is_running() and running:
                loop_start = time.perf_counter()

                # Check for config changes
                changed, new_config = self.config_watcher.check()
                if changed:
                    from .config_watcher import _diff_configs
                    changes = _diff_configs(self.retarget_config, new_config)
                    self._reload_retargeter(new_config, changes)
                    # Re-process current frame with new config
                    self.retargeter.reset()
                    if last_result is not None:
                        raw_kp = data[current_frame].get(hand_key)
                        if raw_kp is not None and not np.allclose(raw_kp, 0):
                            last_result = self._process_frame(raw_kp)
                            self.data.qpos[:] = last_result["qpos"]
                            mujoco.mj_forward(self.model, self.data)

                # Clear highlight if expired
                self._check_highlight_timeout()

                if not paused:
                    # Get frame data
                    frame_data = data[current_frame]
                    raw_kp = frame_data.get(hand_key)

                    if raw_kp is not None and not np.allclose(raw_kp, 0):
                        last_result = self._process_frame(raw_kp)

                        # Set robot qpos
                        self.data.qpos[:] = last_result["qpos"]
                        mujoco.mj_forward(self.model, self.data)

                    # Advance frame
                    current_frame = (current_frame + 1) % total_frames

                # Draw skeletons
                if last_result is not None:
                    with viewer.lock():
                        self.drawer.draw(
                            viewer.user_scn,
                            mediapipe_kp=last_result["mediapipe_kp"],
                            scaled_kp=last_result["scaled_kp"],
                            pinch_alphas=last_result.get("pinch_alphas"),
                        )

                viewer.sync()

                # Frame rate control
                elapsed = time.perf_counter() - loop_start
                if elapsed < frame_time:
                    time.sleep(frame_time - elapsed)

        signal.signal(signal.SIGINT, old_handler)
        print(f"\nViewer closed at frame {current_frame}")

    def view_single_frame(
        self,
        raw_keypoints: np.ndarray,
    ):
        """View a single frame interactively.

        Args:
            raw_keypoints: (21, 3) raw MediaPipe keypoints
        """
        result = self._process_frame(raw_keypoints)

        self.data.qpos[:] = result["qpos"]
        mujoco.mj_forward(self.model, self.data)

        print("Viewing single frame. Close viewer window to exit.")
        print(f"Edit {self.retarget_config_path.name} to tune parameters (auto-reload)")

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self._set_camera(viewer)

            while viewer.is_running():
                # Check config changes
                changed, new_config = self.config_watcher.check()
                if changed:
                    self._reload_retargeter(new_config, [])
                    self.retargeter.reset()
                    result = self._process_frame(raw_keypoints)
                    self.data.qpos[:] = result["qpos"]
                    mujoco.mj_forward(self.model, self.data)

                self._check_highlight_timeout()

                with viewer.lock():
                    self.drawer.draw(
                        viewer.user_scn,
                        mediapipe_kp=result["mediapipe_kp"],
                        scaled_kp=result["scaled_kp"],
                        pinch_alphas=result.get("pinch_alphas"),
                    )

                viewer.sync()
                time.sleep(0.033)  # ~30fps


__all__ = ["TuningViewer"]

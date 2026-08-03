"""Teleoperation with Real Wuji Hand Hardware.

Uses the Retargeter interface to map hand tracking input to Wuji Hand joint angles,
sent to real hardware via wujihandpy.

Usage:
    # Simple run with default (replay data/avp1.pkl)
    python teleop_real.py

    # Replay MediaPipe recording
    python teleop_real.py --play data/avp1.pkl

    # MP4 video input with MediaPipe hand detection
    python teleop_real.py --video data/right.mp4 --hand right
    python teleop_real.py --video data/right.mp4 --hand right --show-video

    # RealSense camera input with MediaPipe hand detection
    python teleop_real.py --realsense --hand right

    # ZED camera input with MediaPipe hand detection
    python teleop_real.py --zed --hand right

    # Live VisionPro input
    python teleop_real.py --input visionpro --ip <your-vision-pro-ip>

    # Record input data
    python teleop_real.py --input visionpro --record

    # Live Wuji Glove input via wuji_sdk
    python teleop_real.py --input wuji_glove --hand right --glove-sn <YOUR_SN>

Input device types:
- visionpro: Live VisionPro input
- mediapipe_replay: Replay recorded MediaPipe hand tracking data
- video: MP4 video input with MediaPipe hand detection
- realsense: RealSense camera input with MediaPipe hand detection
- zed: ZED camera input with MediaPipe hand detection
- wuji_glove: Live Wuji Glove input via wuji_sdk
"""

import argparse
import pickle
import sys
import time
from pathlib import Path

import numpy as np
import wujihandpy
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from wuji_retargeting import Retargeter
from utils.config_paths import resolve_mjcf_path, mjcf_joint_order, qpos_reorder_perm
from input_devices.visionpro import VisionPro
from input_devices.mediapipe_replay import MediaPipeReplay
try:
    from input_devices.video_mediapipe import VideoMediaPipe
except ImportError:
    VideoMediaPipe = None
try:
    from input_devices.realsense_mediapipe import RealsenseMediaPipe
except ImportError:
    RealsenseMediaPipe = None
try:
    from input_devices.zed_mediapipe import ZedMediaPipe
except ImportError:
    ZedMediaPipe = None
try:
    from input_devices.wuji_glove_device import WujiGloveDevice
    WUJI_SDK_AVAILABLE = True
except ImportError:
    WujiGloveDevice = None
    WUJI_SDK_AVAILABLE = False


# =============================================================================
# Hardware backends.
#
# Wuji Hand connects over USB-CDC via wujihandpy.Hand; Wuji Hand 2 is a networked
# "Wuji Hand 2" reached over Ethernet via wuji_sdk.SdkManager. Both are wrapped
# behind a uniform { send(qpos), close() } interface so the teleop loop dispatches
# on --hand-model without caring about the transport. close() is called from a
# finally block so Ctrl+C also runs cleanup and the Wuji Hand 2 session is not leaked.
# =============================================================================
from typing import Optional


# NOTE: per-joint calibration (sign/offset) is intentionally NOT exposed in the
# public release. Feeding an unvalidated sign flip / offset straight to the MIT
# controller can drive a real joint in the wrong direction or past its safe range,
# and it cannot be previewed in simulation — too dangerous for an end-user knob.
# Wuji Hand 2 runs with the validated default joint mapping (firmware == retargeted qpos).


def _set_with_retry(desc, fn, attempts=3, backoff=0.6):
    """Call fn() (a device-config SET), retrying on transient SDK timeouts.

    Right after connect — especially when a handedness probe just churned the
    shared 'wuji_hand_2' bridge by connecting/disconnecting another hand (only
    happens with 2+ hands online) — the first config SET can hit "device not
    responding" while the old bridge is still tearing down. These SETs are
    idempotent, so a few short retries recover it. Bounded (default 3 attempts)
    so a genuinely dead hand still fails fast, and the success path takes the
    first attempt with no added delay.
    """
    for i in range(attempts):
        try:
            return fn()
        except (AttributeError, TypeError):
            # Not a transient bridge timeout — a missing accessor or a bad
            # argument type is permanent, so surface it immediately instead of
            # masking it as "not responding" through three doomed retries.
            raise
        except Exception:
            if i == attempts - 1:
                raise
            print(f"Wuji Hand 2 init: {desc} not responding "
                  f"(attempt {i + 1}/{attempts}); bridge settling, retrying in {backoff}s...")
            time.sleep(backoff)


class WujiHandBackend:
    """Hardware backend for Wuji Hand via wujihandpy (USB-CDC)."""

    def __init__(self, hand_serial: str = ""):
        self._hand = (
            wujihandpy.Hand(serial_number=hand_serial) if hand_serial else wujihandpy.Hand()
        )
        self._hand.write_joint_enabled(True)
        self._controller = self._hand.realtime_controller(
            enable_upstream=False,
            filter=wujihandpy.filter.LowPass(cutoff_freq=5.0),
        )
        time.sleep(0.5)

    def send(self, qpos: np.ndarray) -> None:
        self._controller.set_joint_target_position(qpos.reshape(5, 4))

    def close(self) -> None:
        if getattr(self, "_hand", None) is not None:
            self._hand.write_joint_enabled(False)


class WujiHand2Backend:
    """Hardware backend for Wuji Hand 2 via wuji_sdk (networked Wuji Hand 2)."""

    _ENABLE_TIMEOUT_SEC = 5.0
    # status_word ext_state after a successful enable(): 0=Init 1=Ready 2=Enabled 3=Stopped
    _READY_EXT_STATE = 2

    def __init__(
        self,
        ip: str,
        kp: float,
        kd: float,
        current_limit: float,
        handedness: Optional[str] = None,
    ):
        try:
            import wuji_sdk
            from wuji_sdk import SdkManager
        except ImportError as e:
            raise RuntimeError(
                "wuji_sdk not importable. Install the Wuji Hand 2 SDK wheel."
            ) from e

        self._sdk = wuji_sdk
        self._manager = SdkManager.instance()

        # Connection: explicit --wuji-hand-2-ip wins. Otherwise scan for Wuji Hand 2
        # devices (SN starts "WH"; gloves are "WG"). One hand -> connect it. Several
        # hands online (bimanual) -> pick the one whose reported handedness matches
        # --hand. We can't use connect(handedness=) directly (it also matches the
        # same-side glove), and DiscoveredDevice carries no handedness, so we probe:
        # connect each hand, read hand.handedness(), keep the match, disconnect the
        # rest. Probing only opens a session (no enable / motion).
        t0 = time.monotonic()
        if ip:
            print(f"Wuji Hand 2 init: connect by address ip={ip}")
            self._hand = self._manager.connect(address=ip, device_name="wuji_hand_2")
        else:
            print("Wuji Hand 2 init: no --wuji-hand-2-ip; scanning for a wuji_hand_2 on the network...")
            # "WH" is the hardware serial-number prefix burned into the device (not a product
            # codename); scan() filters discovered devices by it. Keep as-is.
            hands = [d for d in self._manager.scan() if str(d.sn).upper().startswith("WH")]
            if not hands:
                raise RuntimeError("no Wuji Hand 2 found on the network; check power/network.")
            if len(hands) == 1:
                d = hands[0]
                print(f"Wuji Hand 2 init: discovered {d.sn} at {d.address}")
                self._hand = self._manager.connect(address=d.address, device_name="wuji_hand_2")
            else:
                want = (handedness or "").lower()
                if want not in ("left", "right"):
                    listing = ", ".join(f"{d.sn}@{d.address}" for d in hands)
                    raise RuntimeError(
                        f"{len(hands)} hands online ({listing}); pass --hand left/right "
                        f"or --wuji-hand-2-ip <address:port>."
                    )
                print(f"Wuji Hand 2 init: {len(hands)} hands online; selecting the {want} one by handedness...")
                self._hand = None
                for d in hands:
                    h = self._manager.connect(address=d.address, device_name="wuji_hand_2")
                    try:
                        side = str(h.handedness().get()).lower()
                    except Exception:
                        side = "?"
                    if side == want:
                        print(f"Wuji Hand 2 init: matched {want} hand {d.sn} at {d.address}")
                        self._hand = h
                        break
                    print(f"Wuji Hand 2 init:   {d.sn} reports '{side}', skipping")
                    h.disconnect()
                    time.sleep(0.2)  # let the device_name bridge fully release
                if self._hand is None:
                    listing = ", ".join(str(d.sn) for d in hands)
                    raise RuntimeError(
                        f"no {want} Wuji Hand 2 among [{listing}]; pass --wuji-hand-2-ip <address:port>."
                    )
        print(
            f"Wuji Hand 2 init: connect done in {time.monotonic() - t0:.3f}s "
            f"sn={self._hand.serial_number}"
        )

        # From here on the session is open. If any setup step fails, disconnect
        # before propagating — otherwise the session is left on the hand and the
        # next run fails with "Session already exists (0x0013)".
        try:
            n_online = self._hand.online_joints_count().get()
            if n_online == 0:
                raise RuntimeError("Wuji Hand 2: 0 joints online — check device power/network")
            print(f"Wuji Hand 2 connected: {n_online}/20 joints online")

            # Settle before configuring the controller.
            time.sleep(0.5)

            # These config SETs are retried (bounded) because the first one after a
            # multi-hand handedness probe can transiently time out while the churned
            # shared bridge settles. enable() below is intentionally NOT retried.
            # The firmware defaults to MIT control mode, so no control-mode SET is
            # needed (there is no device-level control_mode accessor in the SDK).
            #
            # effort_limit is the per-joint current cap in amps (Kt=1.0 placeholder,
            # so effort in N·m == current in A for now); a scalar broadcasts to all
            # 20 joints.
            _set_with_retry(
                "effort_limit",
                lambda: self._hand.effort_limit().set(current_limit),
            )
            # A single (kp, kd) tuple broadcasts the same MIT gains to all 20 joints.
            _set_with_retry(
                "mit_params",
                lambda: self._hand.mit_params().set((kp, kd)),
            )
            self._hand.enable()
            print(f"Wuji Hand 2 init: enable() sent")

            # Wait until all live joints reach ext_state=2 (Enabled). Diagnostics
            # arrive as a subscription stream now; recv() is non-blocking and
            # returns the latest frame or None. The frame carries online joints
            # only (variable length, looked up by nid) — no None padding.
            deadline = time.monotonic() + self._ENABLE_TIMEOUT_SEC
            enabled = False
            last_frame = None
            diag_sub = self._hand.joint_diagnostics().subscribe()
            try:
                while time.monotonic() < deadline:
                    time.sleep(0.2)
                    frame = diag_sub.recv()
                    if frame is None or not frame.joints:
                        continue
                    last_frame = frame
                    # Filter on vbus_v_fb > 0.5 (live inverter). A joint with a
                    # dead inverter still reports diagnostics with vbus=0 /
                    # ext_state=0 and would otherwise block the ready check forever.
                    live = [e for e in frame.joints if e.vbus_v_fb > 0.5]
                    if live and all(
                        e.status_word.ext_state == self._READY_EXT_STATE for e in live
                    ):
                        enabled = True
                        break
            finally:
                diag_sub.close()
            if not enabled:
                print("Wuji Hand 2: enable timeout. Per-joint state:")
                for e in (last_frame.joints if last_frame is not None else []):
                    print(
                        f"  nid={e.nid}: ext_state={e.status_word.ext_state} "
                        f"({e.status_word.ext_state_name}) vbus={e.vbus_v_fb:.2f}"
                    )
                self._hand.disable()
                raise RuntimeError(f"Wuji Hand 2: enable timeout after {self._ENABLE_TIMEOUT_SEC}s")
            print(f"Wuji Hand 2 enabled (kp={kp}, kd={kd}, current_limit={current_limit}A)")
            # publisher creation is post-enable; keep it INSIDE this guard so a
            # failure here also tears the session down instead of leaking it.
            self._publisher = self._hand.joint_command().publish()
            self._JointCommand = self._sdk.JointCommand
        except BaseException:
            # Best-effort teardown so a failed init doesn't leak the session.
            try:
                self._manager.disconnect_all()
            except Exception:
                pass
            raise

    def send(self, qpos: np.ndarray) -> None:
        # Send the retargeted qpos straight through — no per-joint calibration is
        # applied in the public path (see the module note above).
        positions = qpos.astype(np.float64).tolist()
        # MIT: one JointCommand per joint (AOS) — position target with zero
        # feedforward velocity / effort.
        self._publisher.send([self._JointCommand(p, 0.0, 0.0) for p in positions])

    def close(self) -> None:
        if getattr(self, "_publisher", None) is not None:
            self._publisher.close()
        if getattr(self, "_hand", None) is not None:
            self._hand.disable()
        if getattr(self, "_manager", None) is not None:
            self._manager.disconnect_all()


def _infer_hand_model(config_path) -> str:
    """Infer 'wuji_hand_2' vs 'wuji_hand' from a retarget config.

    A Wuji Hand 2 config overrides the optimizer's hand assets (``urdf_path`` /
    ``mjcf_path``) to the Wuji Hand 2 model, whose assets live under ``hand2/``
    in the wuji-description package. Anything else is treated as the default
    Wuji Hand. Used only when ``--hand-model`` is omitted, so passing a Wuji
    Hand 2 config selects that backend without a separate flag.
    """
    try:
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return "wuji_hand"
    opt = cfg.get("optimizer") or {}
    blob = " ".join(
        str(x) for x in (opt.get("urdf_path"), opt.get("mjcf_path"), config_path)
    )
    # Key off the Wuji Hand 2 asset directory (``hand2/``) rather than the file
    # name: the asset path stays stable even if the config file is renamed.
    return "wuji_hand_2" if "hand2" in blob.lower() else "wuji_hand"


def run_tuning_mode(
    hand_side: str,
    config_path: str,
    input_device_type: str,
    mediapipe_replay_path: str = "",
    video_path: str = "",
    show_video: bool = False,
    viz_config_path: str = None,
    fps: float = 30.0,
    device_name: str = "glove",
    glove_sn: str = "",
):
    """Backward-compatible wrapper for the standalone tuning tool."""
    import tuning_tool

    class TuningArgs:
        pass

    args = TuningArgs()
    args.hand = hand_side
    args.config = config_path
    args.viz_config = viz_config_path
    args.play = mediapipe_replay_path or None
    args.video = video_path or None
    args.realsense = input_device_type == "realsense"
    args.zed = input_device_type == "zed"
    args.wuji_glove = input_device_type == "wuji_glove"
    args.device_name = device_name
    args.glove_sn = glove_sn
    args.show_video = show_video
    args.fps = fps

    print(
        "Note: --tuning on teleop scripts is kept for compatibility. "
        "Prefer running tuning_tool.py directly."
    )

    if args.wuji_glove:
        tuning_tool.run_wuji_glove_mode(args)
    elif args.zed:
        tuning_tool.run_zed_mode(args)
    elif args.realsense:
        tuning_tool.run_realsense_mode(args)
    elif args.video:
        tuning_tool.run_video_mode(args)
    elif args.play:
        tuning_tool.run_recording_mode(args)
    else:
        print("Tuning mode requires --play FILE or a live/video input source")


def run_teleop(
    hand_side: str = "right",
    config_path: str = "config/adaptive_analytical_avp.yaml",
    input_device_type: str = "mediapipe_replay",
    visionpro_ip: str = "192.168.50.127",
    mediapipe_replay_path: str = "data/avp1.pkl",
    playback_speed: float = 1.0,
    playback_loop: bool = True,
    enable_recording: bool = False,
    video_path: str = "",
    show_video: bool = False,
    device_name: str = "glove",
    glove_sn: str = "",
    # Hardware backend selection + Wuji Hand 2 connection params.
    hand_model: str = "wuji_hand",
    hand_serial: str = "",
    wuji_hand_2_ip: str = "",
    kp: float = 3.0,
    kd: float = 0.1,
    current_limit: float = 1.5,
):
    """Run teleoperation with real hardware.

    Args:
        hand_side: 'right' or 'left'
        config_path: Path to YAML configuration file
        input_device_type: Input device type ('visionpro' or 'mediapipe_replay')
        visionpro_ip: VisionPro IP address
        mediapipe_replay_path: Path to MediaPipe recording (.pkl)
        playback_speed: Playback speed for replay mode
        playback_loop: Whether to loop replay
        enable_recording: Whether to record raw input data
        video_path: Path to MP4 video file
        show_video: Whether to display video with MediaPipe landmarks overlay
        device_name: wuji_sdk device name for Wuji Glove (default "glove")
        glove_sn: Wuji Glove serial number (required when multiple Wuji devices online)
    """
    hand_side = hand_side.lower()
    assert hand_side in {"right", "left"}, "hand_side must be 'right' or 'left'"
    assert hand_model in {"wuji_hand", "wuji_hand_2"}, f"hand_model must be 'wuji_hand' or 'wuji_hand_2', got {hand_model}"

    # backend/input_device are set up front so the finally can always check them and
    # so a failure during setup never leaves a reference unbound. The hand is
    # energized LAST (just before the loop, inside the try) — see below.
    backend = None
    input_device = None

    # Load config to get video_input settings if needed
    config_file = Path(__file__).parent / config_path
    with open(config_file, 'r') as f:
        full_config = yaml.safe_load(f)
    video_config = full_config.get('video_input', {})

    def create_wuji_glove_device():
        if not WUJI_SDK_AVAILABLE:
            raise ImportError(
                "wuji_sdk is not installed. "
                "Please install wuji_sdk to use --input wuji_glove."
            )
        return WujiGloveDevice(
            hand_side=hand_side,
            device_name=device_name,
            sn=glove_sn or None,
        )

    # Initialize input device
    device_map = {
        "visionpro": lambda: VisionPro(ip=visionpro_ip),
        "mediapipe_replay": lambda: MediaPipeReplay(
            record_path=mediapipe_replay_path,
            playback_speed=playback_speed,
            loop=playback_loop,
        ),
        "video": lambda: VideoMediaPipe(
            video_path=video_path,
            hand_side=hand_side,
            playback_speed=playback_speed,
            loop=playback_loop,
            video_config=video_config,
            show_video=show_video,
        ),
        "realsense": lambda: RealsenseMediaPipe(
            hand_side=hand_side,
            video_config=video_config,
            show_video=show_video,
        ),
        "zed": lambda: ZedMediaPipe(
            hand_side=hand_side,
            video_config=video_config,
            show_video=show_video,
        ),
        "wuji_glove": create_wuji_glove_device,
    }
    if input_device_type not in device_map:
        raise ValueError(f"Unknown input device type: {input_device_type}")

    if input_device_type == "mediapipe_replay" and not mediapipe_replay_path:
        raise ValueError("mediapipe_replay_path is required for mediapipe_replay mode")
    if input_device_type == "video" and not video_path:
        raise ValueError("video_path is required for video mode")
    if input_device_type == "video" and VideoMediaPipe is None:
        raise ImportError("video mode requires mediapipe and opencv-python")
    if input_device_type == "realsense" and RealsenseMediaPipe is None:
        raise ImportError("realsense mode requires mediapipe, opencv-python, and pyrealsense2")
    if input_device_type == "zed" and ZedMediaPipe is None:
        raise ImportError("zed mode requires mediapipe, opencv-python, and pyzed")

    input_device = device_map[input_device_type]()

    # Initialize retargeter
    retargeter = Retargeter.from_yaml(str(config_file), hand_side)

    # qpos comes out in the URDF/Pinocchio joint order, which can differ from the
    # device's order (e.g. Wuji Hand 2 declares fingers index-first). Remap by joint
    # name to the MJCF joint order (== the device's finger1..5 indexing) so the
    # commanded angles land on the right joints. Identity when orders match (Wuji Hand)
    # or when the config sets no mjcf_path.
    mjcf_path = resolve_mjcf_path(config_file)
    _qpos_perm = qpos_reorder_perm(
        retargeter.optimizer.robot.dof_joint_names,
        mjcf_joint_order(mjcf_path),
    )
    # A declared optimizer.mjcf_path means a custom hand whose joint order MUST be
    # remapped; if the names can't be aligned, qpos_reorder_perm returns None — the
    # SAME value as the legitimate Wuji Hand "no mjcf_path" case. Driving the hand on
    # that ambiguous None would silently move the wrong joints, so fail loudly here.
    # (Wuji Hand has no mjcf_path -> mjcf_path is None -> this is skipped.)
    if mjcf_path is not None and _qpos_perm is None:
        raise ValueError(
            "config declares optimizer.mjcf_path but the URDF<->MJCF joint names "
            "could not be aligned; refusing to drive the hand with an unverified "
            "joint order (would move the wrong joints). Check optimizer.link_naming "
            "and that urdf_path and mjcf_path describe the same hand.\n"
            f"  mjcf: {mjcf_path}"
        )
    if _qpos_perm is not None:
        print(f"qpos remap active: URDF order -> MJCF/device order ({_qpos_perm.tolist()})")
    else:
        print("qpos remap: identity (no optimizer.mjcf_path; Wuji Hand-style hand)")

    # Disable recording when using replay mode
    if input_device_type == "mediapipe_replay" and enable_recording:
        print("Note: Recording disabled in replay mode")
        enable_recording = False

    # Prepare recording
    input_data_log = [] if enable_recording else None
    start_time = time.time()

    try:
        # Energize the hand LAST — after every fallible setup step above — and INSIDE
        # this try, so the finally always runs close() (and cleans the input device)
        # even if bring-up itself fails.
        if hand_model == "wuji_hand_2":
            backend = WujiHand2Backend(
                ip=wuji_hand_2_ip,
                kp=kp,
                kd=kd,
                current_limit=current_limit,
                handedness=hand_side,
            )
        else:
            backend = WujiHandBackend(hand_serial=hand_serial)

        print(f"Starting teleoperation...")
        print(f"  Config: {config_path}")
        print(f"  Hand: {hand_side}")
        print(f"  Input: {input_device_type}")
        print(f"  Recording: {'ON' if enable_recording else 'OFF'}")
        print("=" * 50)

        frame_count = 0
        fps_start_time = time.time()

        while True:
            # Get finger data
            fingers_data = input_device.get_fingers_data()
            fingers_pose = fingers_data[f"{hand_side}_fingers"]  # (21, 3)

            # Skip until the first valid frame arrives from the input device.
            if fingers_pose is None or np.allclose(fingers_pose, 0):
                time.sleep(0.01)
                continue

            # Record raw input data if enabled
            if enable_recording:
                input_data_log.append({
                    "t": time.time() - start_time,
                    "left_fingers": (
                        None
                        if fingers_data["left_fingers"] is None
                        else fingers_data["left_fingers"].copy()
                    ),
                    "right_fingers": (
                        None
                        if fingers_data["right_fingers"] is None
                        else fingers_data["right_fingers"].copy()
                    ),
                })

            # Retarget to joint angles
            qpos = retargeter.retarget(fingers_pose)  # (20,)

            # FPS counter
            frame_count += 1
            if frame_count % 100 == 0:
                elapsed = time.time() - fps_start_time
                fps = frame_count / elapsed
                print(f"FPS: {fps:.1f}")

            # Send to hardware via backend (Wuji Hand or Wuji Hand 2, decided at init).
            backend.send(qpos if _qpos_perm is None else qpos[_qpos_perm])


    except KeyboardInterrupt:
        print("\nStopping controller...")
    finally:
        # SIGINT-safe cleanup: close backend + input device, each None-guarded (a
        # failure during setup or bring-up can leave either unbound).
        if backend is not None:
            try:
                backend.close()
            except Exception as _e:
                print(f"backend.close() raised: {type(_e).__name__}: {_e}")
        if input_device is not None:
            for method_name in ("stop", "cleanup", "close"):
                method = getattr(input_device, method_name, None)
                if callable(method):
                    try:
                        method()
                    except Exception:
                        pass

    return input_data_log


def main():
    parser = argparse.ArgumentParser(
        description='Teleoperation with Real Wuji Hand Hardware',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Simple run with default (replay data/avp1.pkl)
  python teleop_real.py

  # Replay MediaPipe recording
  python teleop_real.py --play data/avp1.pkl

  # MP4 video input with MediaPipe hand detection
  python teleop_real.py --video data/right.mp4 --hand right
  python teleop_real.py --video data/right.mp4 --hand right --show-video

  # RealSense camera input with MediaPipe hand detection
  python teleop_real.py --realsense --hand right

  # ZED camera input with MediaPipe hand detection
  python teleop_real.py --zed --hand right

  # Live VisionPro input
  python teleop_real.py --input visionpro --ip <your-vision-pro-ip>

  # Record input data while using VisionPro
  python teleop_real.py --input visionpro --record

  # Compatibility tuning shortcut
  python teleop_real.py --play data/avp1.pkl --hand right --tuning
        """
    )

    # Config
    parser.add_argument('--config', type=str, default='config/adaptive_analytical_avp.yaml',
                        help='Path to YAML configuration file')
    parser.add_argument('--hand', type=str, default='right', choices=['left', 'right'],
                        help='Hand side (default: right)')

    # Input device options
    parser.add_argument('--input', type=str, default=None,
                        choices=['visionpro', 'mediapipe_replay', 'video', 'realsense', 'zed', 'wuji_glove'],
                        help='Input device type')

    # Shortcut options
    parser.add_argument('--play', type=str, default=None, metavar='FILE',
                        help='Play MediaPipe recording file (shortcut for --input mediapipe_replay)')

    # VisionPro options
    parser.add_argument('--ip', type=str, default='192.168.50.127',
                        help='VisionPro IP address (default: 192.168.50.127)')

    # Playback options
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed for replay mode (default: 1.0)')
    parser.add_argument('--no-loop', action='store_true',
                        help='Disable looping for replay mode')

    # Recording
    parser.add_argument('--record', action='store_true',
                        help='Record input data to file')
    parser.add_argument('--output', type=str, default=None, metavar='FILE',
                        help='Output file for recording (default: auto-generated)')
    parser.add_argument('--video', type=str, default=None, metavar='FILE',
                        help='Play MP4 video file with MediaPipe hand detection (shortcut for --input video)')
    parser.add_argument('--realsense', action='store_true',
                        help='Use RealSense camera with MediaPipe hand detection (shortcut for --input realsense)')
    parser.add_argument('--zed', action='store_true',
                        help='Use ZED camera with MediaPipe hand detection (shortcut for --input zed)')
    parser.add_argument('--show-video', action='store_true',
                        help='Show video with MediaPipe landmarks overlay (video/realsense/zed mode)')
    parser.add_argument('--tuning', action='store_true',
                        help='Launch tuning visualization mode (deprecated; use tuning_tool.py directly)')
    parser.add_argument('--viz-config', type=str, default=None,
                        help='Path to tuning visualization config (default: config/tuning_viz.yaml)')
    parser.add_argument('--fps', type=float, default=30.0,
                        help='Playback FPS for tuning mode (default: 30)')
    parser.add_argument('--device-name', type=str, default='glove',
                        help='wuji_sdk device name for Wuji Glove (default: glove)')
    parser.add_argument('--glove-sn', type=str, default='',
                        help='Wuji Glove serial number (required when multiple Wuji devices online)')

    # Hardware backend selection + Wuji Hand 2 connection params.
    parser.add_argument('--hand-model', type=str, default=None, choices=['wuji_hand', 'wuji_hand_2'],
                        help='Hand hardware model: wuji_hand (USB via wujihandpy) or wuji_hand_2 (networked via wuji_sdk). '
                             'Default: inferred from the config (Wuji Hand 2 if it overrides the hand to a Wuji Hand 2 model, else Wuji Hand).')
    parser.add_argument('--hand-serial', type=str, default='',
                        help='Wuji Hand hand serial number (for wujihandpy.Hand selection)')
    parser.add_argument('--wuji-hand-2-ip', type=str, default='',
                        help='Wuji Hand 2 SDK address, e.g. 192.168.1.111:50001 (run a scan to find it)')
    parser.add_argument('--kp', type=float, default=3.0, help='Wuji Hand 2 MIT kp (default: 3.0)')
    parser.add_argument('--kd', type=float, default=0.1, help='Wuji Hand 2 MIT kd (default: 0.1)')
    parser.add_argument('--current-limit', type=float, default=1.5,
                        help='Wuji Hand 2 per-joint current limit in amps (SDK effort_limit, default: 1.5)')
    # NOTE: per-joint calibration (sign/offset) is deliberately not exposed here —
    # an unvalidated sign flip / offset goes straight to the MIT controller on real
    # hardware (and can't be previewed in sim), which is unsafe as a user knob.

    args = parser.parse_args()

    # Determine input device type and paths
    input_device_type = args.input
    mediapipe_replay_path = ""
    video_path = ""

    if args.zed:
        input_device_type = "zed"
    elif args.realsense:
        input_device_type = "realsense"
    elif args.video:
        input_device_type = "video"
        video_path = args.video
    elif args.play:
        input_device_type = "mediapipe_replay"
        mediapipe_replay_path = args.play

    # Default to mediapipe_replay with data/avp1.pkl if no input specified
    if input_device_type is None:
        input_device_type = "mediapipe_replay"
        mediapipe_replay_path = "data/avp1.pkl"

    # Auto-switch config for non-AVP input devices. For the Wuji Glove, pick the
    # Wuji Hand 2 config when --hand-model wuji_hand_2 so the IK hand (urdf_path/mjcf_path in
    # that config) matches the physical hand being driven.
    if args.config == 'config/adaptive_analytical_avp.yaml':
        if input_device_type in ("realsense", "video", "zed"):
            args.config = 'config/adaptive_analytical_video.yaml'
        elif input_device_type == "wuji_glove":
            suffix = "wuji_hand_2_" if args.hand_model == "wuji_hand_2" else ""
            args.config = f'config/adaptive_analytical_wuji_glove_{suffix}{args.hand}.yaml'

    # Resolve hand model: explicit --hand-model wins; otherwise infer from the
    # config so that passing a Wuji Hand 2 config (e.g. ..._wuji_hand_2_right.yaml) selects
    # the Wuji Hand 2 network backend without also needing --hand-model wuji_hand_2.
    if args.hand_model is None:
        args.hand_model = _infer_hand_model(Path(__file__).parent / args.config)
        print(f"--hand-model not given; inferred '{args.hand_model}' from {args.config}")

    # Compatibility tuning mode. Prefer invoking tuning_tool.py directly.
    if args.tuning:
        viz_config_path = args.viz_config
        if viz_config_path is None:
            default_viz = Path(__file__).parent / "config" / "tuning_viz.yaml"
            if default_viz.exists():
                viz_config_path = "config/tuning_viz.yaml"
        run_tuning_mode(
            hand_side=args.hand,
            config_path=args.config,
            input_device_type=input_device_type,
            mediapipe_replay_path=mediapipe_replay_path,
            video_path=video_path,
            show_video=args.show_video,
            viz_config_path=viz_config_path,
            fps=args.fps,
            device_name=args.device_name,
            glove_sn=args.glove_sn,
        )
        return

    # Run teleoperation
    log = run_teleop(
        hand_side=args.hand,
        config_path=args.config,
        input_device_type=input_device_type,
        visionpro_ip=args.ip,
        mediapipe_replay_path=mediapipe_replay_path,
        playback_speed=args.speed,
        playback_loop=not args.no_loop,
        enable_recording=args.record,
        video_path=video_path,
        show_video=args.show_video,
        device_name=args.device_name,
        glove_sn=args.glove_sn,
        hand_model=args.hand_model,
        hand_serial=args.hand_serial,
        wuji_hand_2_ip=args.wuji_hand_2_ip,
        kp=args.kp,
        kd=args.kd,
        current_limit=args.current_limit,
    )

    # Save recording if enabled
    if log is not None and len(log) > 0:
        if args.output:
            log_path = Path(args.output)
        else:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            log_path = Path(__file__).parent / f"input_data_log_{timestamp}.pkl"

        with open(log_path, "wb") as f:
            pickle.dump(log, f)
        print(f"Saved input data log with {len(log)} entries to {log_path}")


if __name__ == "__main__":
    main()

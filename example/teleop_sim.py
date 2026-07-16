"""Teleoperation with MuJoCo Simulation.

Uses the Retargeter interface to map hand tracking input to Wuji Hand joint angles,
visualized in MuJoCo simulation.

Usage:
    # Replay MediaPipe recording (default)
    mjpython teleop_sim.py --play data/avp1.pkl --hand left

    # MP4 video input with MediaPipe hand detection
    mjpython teleop_sim.py --video data/right.mp4 --hand right
    mjpython teleop_sim.py --video data/right.mp4 --hand right --show-video

    # RealSense camera input with MediaPipe hand detection
    mjpython teleop_sim.py --realsense --hand right

    # ZED camera input with MediaPipe hand detection
    mjpython teleop_sim.py --zed --hand right

    # Live VisionPro input
    mjpython teleop_sim.py --input visionpro --ip <your-vision-pro-ip>

    # Record input data while using VisionPro
    mjpython teleop_sim.py --input visionpro --record

    # Live Wuji Glove input via wuji_sdk
    mjpython teleop_sim.py --input wuji_glove --hand right --glove-sn <YOUR_SN>

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
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
EXAMPLE_DIR = Path(__file__).resolve().parent
if str(EXAMPLE_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLE_DIR))

Quest3Device = None


def _resolve_example_path(path: str) -> str:
    candidate = Path(path)
    if candidate.is_absolute():
        return str(candidate)
    return str((Path(__file__).parent / candidate).resolve())


def create_input_device(
    input_device_type: str,
    hand_side: str,
    mediapipe_replay_path: str = "",
    visionpro_ip: str = "192.168.50.127",
    playback_speed: float = 1.0,
    playback_loop: bool = True,
    video_path: str = "",
    show_video: bool = False,
    video_config: dict | None = None,
    device_name: str = "glove",
    glove_sn: str = "",
    quest_host: str = "0.0.0.0",
    quest_port: int = 9001,
    quest_left_config: str = "../configs/quest3_web/webxr_hand_mapping_left.yaml",
    quest_right_config: str = "../configs/quest3_web/webxr_hand_mapping_right.yaml",
    quest_service_config: str = "",
    grip_deadman_threshold: float = 0.5,
):
    if input_device_type == "visionpro":
        from example.input_devices.visionpro import VisionPro

        return VisionPro(ip=visionpro_ip)
    if input_device_type == "mediapipe_replay":
        from example.input_devices.mediapipe_replay import MediaPipeReplay

        return MediaPipeReplay(
            record_path=mediapipe_replay_path,
            playback_speed=playback_speed,
            loop=playback_loop,
        )
    if input_device_type == "video":
        try:
            from example.input_devices.video_mediapipe import VideoMediaPipe
        except ImportError as exc:
            raise ImportError("video mode requires mediapipe and opencv-python") from exc
        return VideoMediaPipe(
            video_path=video_path,
            hand_side=hand_side,
            playback_speed=playback_speed,
            loop=playback_loop,
            video_config=video_config,
            show_video=show_video,
        )
    if input_device_type == "realsense":
        try:
            from example.input_devices.realsense_mediapipe import RealsenseMediaPipe
        except ImportError as exc:
            raise ImportError(
                "realsense mode requires mediapipe, opencv-python, and pyrealsense2"
            ) from exc
        return RealsenseMediaPipe(
            hand_side=hand_side,
            video_config=video_config,
            show_video=show_video,
        )
    if input_device_type == "zed":
        try:
            from example.input_devices.zed_mediapipe import ZedMediaPipe
        except ImportError as exc:
            raise ImportError("zed mode requires mediapipe, opencv-python, and pyzed") from exc
        return ZedMediaPipe(
            hand_side=hand_side,
            video_config=video_config,
            show_video=show_video,
        )
    if input_device_type == "wuji_glove":
        try:
            from example.input_devices.wuji_glove_device import WujiGloveDevice
        except ImportError as exc:
            raise ImportError(
                "wuji_sdk is not installed. "
                "Please install wuji_sdk to use --input wuji_glove."
            ) from exc
        return WujiGloveDevice(
            hand_side=hand_side,
            device_name=device_name,
            sn=glove_sn or None,
        )
    if input_device_type == "quest3":
        device_cls = Quest3Device
        if device_cls is None:
            from example.input_devices.quest3_device import Quest3Device as device_cls
        if quest_service_config:
            return device_cls.from_service_config(
                _resolve_example_path(quest_service_config),
                start_server=True,
            )
        return device_cls(
            host=quest_host,
            port=quest_port,
            left_config=_resolve_example_path(quest_left_config),
            right_config=_resolve_example_path(quest_right_config),
            grip_deadman_threshold=grip_deadman_threshold,
        )
    raise ValueError(f"Unknown input device type: {input_device_type}")


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
    mediapipe_replay_path: str = "",
    visionpro_ip: str = "192.168.50.127",
    playback_speed: float = 1.0,
    playback_loop: bool = True,
    enable_recording: bool = False,
    video_path: str = "",
    show_video: bool = False,
    device_name: str = "glove",
    glove_sn: str = "",
    quest_host: str = "0.0.0.0",
    quest_port: int = 9001,
    quest_left_config: str = "../configs/quest3/quest26_to_mp21_left.yaml",
    quest_right_config: str = "../configs/quest3/quest26_to_mp21_right.yaml",
    quest_service_config: str = "",
    grip_deadman_threshold: float = 0.5,
    require_deadman: bool = False,
):
    """Run teleoperation with MuJoCo simulation.

    Args:
        hand_side: 'right' or 'left'
        config_path: Path to YAML configuration file
        input_device_type: Input device type ('visionpro' or 'mediapipe_replay')
        mediapipe_replay_path: Path to MediaPipe recording (.pkl)
        visionpro_ip: VisionPro IP address
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
    import mujoco
    import mujoco.viewer
    from wuji_retargeting import Retargeter

    # Load MuJoCo model
    mjcf_path = (
        Path(__file__).resolve().parents[1]
        / "wuji_retargeting" / "wuji_hand_description" / "mjcf" / f"{hand_side}.xml"
    )
    if not mjcf_path.exists():
        raise FileNotFoundError(f"MuJoCo model file not found: {mjcf_path}")

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)

    # Initialize control signals
    for i in range(model.nu):
        if model.actuator_ctrllimited[i]:
            ctrl_range = model.actuator_ctrlrange[i]
            data.ctrl[i] = (ctrl_range[0] + ctrl_range[1]) / 2
        else:
            data.ctrl[i] = 0.0

    # Stabilize model
    for _ in range(100):
        mujoco.mj_step(model, data)

    # Launch viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]

    # Load config to get video_input settings if needed
    config_file = Path(__file__).parent / config_path
    with open(config_file, "r") as f:
        full_config = yaml.safe_load(f)
    video_config = full_config.get("video_input", {})

    if input_device_type == "mediapipe_replay" and not mediapipe_replay_path:
        raise ValueError("mediapipe_replay_path is required for mediapipe_replay mode")
    if input_device_type == "video" and not video_path:
        raise ValueError("video_path is required for video mode")

    input_device = create_input_device(
        input_device_type=input_device_type,
        hand_side=hand_side,
        mediapipe_replay_path=mediapipe_replay_path,
        visionpro_ip=visionpro_ip,
        playback_speed=playback_speed,
        playback_loop=playback_loop,
        video_path=video_path,
        show_video=show_video,
        video_config=video_config,
        device_name=device_name,
        glove_sn=glove_sn,
        quest_host=quest_host,
        quest_port=quest_port,
        quest_left_config=quest_left_config,
        quest_right_config=quest_right_config,
        quest_service_config=quest_service_config,
        grip_deadman_threshold=grip_deadman_threshold,
    )

    # Initialize retargeter
    retargeter = Retargeter.from_yaml(str(config_file), hand_side)

    # Disable recording when using replay mode
    if input_device_type == "mediapipe_replay" and enable_recording:
        print("Note: Recording disabled in replay mode")
        enable_recording = False

    # Prepare recording
    input_data_log = [] if enable_recording else None
    start_time = time.time()

    try:
        print(f"Starting teleoperation...")
        print(f"  Config: {config_path}")
        print(f"  Hand: {hand_side}")
        print(f"  Input: {input_device_type}")
        print(f"  Recording: {'ON' if enable_recording else 'OFF'}")
        print("=" * 50)

        frame_count = 0
        fps_start_time = time.time()

        while viewer.is_running():
            # Get finger data
            fingers_data = input_device.get_fingers_data()
            fingers_pose = fingers_data[f"{hand_side}_fingers"]  # (21, 3)
            controller_state = getattr(input_device, "get_controller_state", lambda: None)()
            deadman = True if controller_state is None else controller_state.deadman
            frame_age = getattr(input_device, "get_frame_age_sec", lambda: None)()
            if require_deadman and not deadman:
                time.sleep(0.01)
                continue

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
                print(f"FPS: {fps:.1f} hand={hand_side} deadman={deadman} age={frame_age}")

            # Set control signals
            if len(qpos) == model.nu:
                data.ctrl[:] = qpos
            else:
                min_len = min(len(qpos), model.nu)
                data.ctrl[:min_len] = qpos[:min_len]

            # Step simulation
            mujoco.mj_step(model, data)
            viewer.sync()

            time.sleep(model.opt.timestep)

    except KeyboardInterrupt:
        print("\nStopping controller...")
    finally:
        viewer.close()
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
        description='Teleoperation with MuJoCo Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Replay MediaPipe recording
  mjpython teleop_sim.py --play data/avp1.pkl --hand left

  # MP4 video input with MediaPipe hand detection
  mjpython teleop_sim.py --video data/right.mp4 --hand right
  mjpython teleop_sim.py --video data/right.mp4 --hand right --show-video

  # RealSense camera input with MediaPipe hand detection
  mjpython teleop_sim.py --realsense --hand right

  # ZED camera input with MediaPipe hand detection
  mjpython teleop_sim.py --zed --hand right

  # Live VisionPro input
  mjpython teleop_sim.py --input visionpro --ip <your-vision-pro-ip>

  # Record input data while using VisionPro
  mjpython teleop_sim.py --input visionpro --record

  # Compatibility tuning shortcut
  mjpython teleop_sim.py --play data/avp1.pkl --hand left --tuning
        """
    )

    # Config
    parser.add_argument('--config', type=str, default='config/adaptive_analytical_avp.yaml',
                        help='Path to YAML configuration file (default: config/adaptive_analytical_avp.yaml)')
    parser.add_argument('--hand', type=str, default='left', choices=['left', 'right'],
                        help='Hand side (default: left)')

    # Input device options
    parser.add_argument('--input', type=str, default=None,
                        choices=['visionpro', 'mediapipe_replay', 'video', 'realsense', 'zed', 'wuji_glove', 'quest3'],
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
    parser.add_argument('--quest-host', type=str, default='0.0.0.0',
                        help='Quest3 WebSocket listen host (default: 0.0.0.0)')
    parser.add_argument('--quest-port', type=int, default=9001,
                        help='Quest3 WebSocket listen port (default: 9001)')
    parser.add_argument('--quest-left-config', type=str,
                        default='../configs/quest3_web/webxr_hand_mapping_left.yaml',
                        help='Quest3 left WebXR-to-MP21 config')
    parser.add_argument('--quest-right-config', type=str,
                        default='../configs/quest3_web/webxr_hand_mapping_right.yaml',
                        help='Quest3 right WebXR-to-MP21 config')
    parser.add_argument('--quest-service-config', type=str, default='',
                        help='Quest3 Control PC service config YAML')
    parser.add_argument('--grip-deadman-threshold', type=float, default=0.5,
                        help='Quest3 global grip deadman threshold (default: 0.5)')
    parser.add_argument('--require-deadman', action='store_true',
                        help='Require Quest3 grip deadman before retargeting in sim')

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

    # Default to mediapipe_replay with example data if no input specified
    if input_device_type is None:
        input_device_type = "mediapipe_replay"
        mediapipe_replay_path = "data/avp1.pkl"

    # Auto-switch config for non-AVP input devices
    if args.config == 'config/adaptive_analytical_avp.yaml':
        if input_device_type in ("realsense", "video", "zed"):
            args.config = 'config/adaptive_analytical_video.yaml'
        elif input_device_type == "wuji_glove":
            args.config = f'config/adaptive_analytical_wuji_glove_{args.hand}.yaml'
        elif input_device_type == "quest3":
            args.config = f'../configs/retargeting/adaptive_analytical_quest3_{args.hand}.yaml'

    # Validate paths
    if input_device_type == "mediapipe_replay" and not mediapipe_replay_path:
        parser.error("--play FILE is required for mediapipe_replay mode")

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
        mediapipe_replay_path=mediapipe_replay_path,
        visionpro_ip=args.ip,
        playback_speed=args.speed,
        playback_loop=not args.no_loop,
        enable_recording=args.record,
        video_path=video_path,
        show_video=args.show_video,
        device_name=args.device_name,
        glove_sn=args.glove_sn,
        quest_host=args.quest_host,
        quest_port=args.quest_port,
        quest_left_config=args.quest_left_config,
        quest_right_config=args.quest_right_config,
        quest_service_config=args.quest_service_config,
        grip_deadman_threshold=args.grip_deadman_threshold,
        require_deadman=args.require_deadman,
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

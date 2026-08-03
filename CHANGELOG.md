# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2026.8.3]

### Changed

- Retuned the `mediapipe_rotation` bias in the Wuji Glove example configs (`adaptive_analytical_wuji_glove_{left,right}.yaml` and `adaptive_analytical_wuji_glove_wuji_hand_2_{left,right}.yaml`) for the new per-serial calibration. The manual `wrist_offset_cm` / `thumb_offset_cm` stay disabled because the SDK applies a per-serial calibrated URDF.

### Fixed

- Fixed `teleop_real.py` crashing at startup with `AttributeError` on the Wuji Hand 2 (network) backend after the Wuji SDK 2026.7.1 resource-API redesign. Control behavior is unchanged. Requires Wuji SDK ≥ 2026.7.1 and matching firmware.

## [2026.6.27]

### Changed

- Standardized hardware naming across the docs and example configs to `Wuji Hand 2` / `Wuji Hand` / `Wuji Glove`, fixing inconsistent model references. The Wuji Glove example configs are now `adaptive_analytical_wuji_glove_wuji_hand_2_{left,right}.yaml`.

## [2026.6.15]

### Added

- Added Wuji Hand 2 (network-connected) support to `teleop_real.py`. Select the hand model with `--hand-model {wuji_hand,wuji_hand_2}` (inferred from the config when omitted). The new network hand also accepts `--wuji-hand-2-ip` (auto-discovers when omitted), `--kp`, `--kd`, and `--current-limit`
- Added config-driven hand model selection for retargeting. Point the optimizer at any hand via `optimizer.urdf_path` (IK) and `optimizer.mjcf_path` (simulation). `optimizer.link_naming` maps logical link roles (palm, fingertip, PIP, DIP, MCP) onto the URDF's actual link names, so anatomically named hands work without code changes. Joint commands are remapped by name across the viewer, simulation, and hardware paths, so a URDF that declares fingers in a different order still drives the right ones. Ships right- and left-hand Wuji Hand 2 configs. The default Wuji Hand path is unchanged when no override is set
- Documented Docker usage in the README. No official Dockerfile is shipped. The Wuji SDK reads per-device assets from `~/.wuji`, which must be mounted into the container when using Wuji Glove or real hardware (simulation and replay do not need it)

### Fixed

- Fixed the adaptive analytical optimizer hardcoding finger joint indices, which could silently apply hyperextension and DIP/PIP coupling constraints to the wrong joints on custom hand URDFs. Indices are now resolved from the kinematic chain, with a clear error at load time if the expected finger links are missing. Behavior on the default hand is unchanged
- Fixed `pip install .` silently producing an `UNKNOWN` package with no dependencies on systems with older setuptools. On Ubuntu 22.04 also run `pip install -U pip` for the fix to take effect — see the Ubuntu note in the README

## [2026.6.10]

### Fixed

- Fixed the `wuji-description` hand model submodule being absent from the v2026.05.26 release. A fresh `git clone --recurse-submodules` now correctly fetches the URDF/MJCF assets, so `teleop_sim`, `teleop_real`, and `tuning_tool` work out of the box.

## [2026.05.26]

### Added

- Added Retargeting Parameter Tuning Guide covering tuning_tool usage, skeleton color legend, a full parameter quick-reference table, and the recommended tuning order

### Changed

- Swapped the hand description submodule from `wuji_hand_description` to `wuji-description`. The URDF and MJCF assets now live under `wuji-description/hand/body/`, and the package no longer depends on the `mujoco-sim` submodule for hand model files.

## [2026.05.23]

### Changed

- Tuned the Wuji Glove left/right example configs for better fit: refined per-finger segment scaling, adjusted MediaPipe rotation calibration, and disabled the wrist/thumb offsets in favor of the SDK's built-in per-device calibration

## [2026.05.18]

### Added

- Added Custom Input Device Integration Guide describing how to integrate a custom hand input device with a small wrapper class
- Added `Custom Input Devices` section to README linking to the new integration guide
- Added Wuji Glove input device (`wuji_glove_device.py`) with SDK-driven 21 MediaPipe keypoint output
- Added Wuji Glove example configs for left and right hands
- Added offset calibration tool (`calibrate_offset.py`) for neutral-pose wrist/thumb offsets
- Added `--input wuji_glove` support in `example/teleop_sim.py` and `example/teleop_real.py`
- Added Wuji Glove live mode to `example/tuning_tool.py`
- Added optimizer extensions for thumb PIP skipping, anti-hyperextension, DIP/PIP coupling, optional URDF override, and retarget keypoint offsets
- Added `wuji-sdk>=0.10.0` dependency and optional `tuning` extra

### Changed

- Updated docs to recommend Wuji Glove as the live input path while keeping other input sources available

## [2026.04.27] - 2026-04-27

### Added

- Added a key-vector retargeting optimizer with example configs for Apple Vision Pro and video
- Added ZED camera support as a real-time hand input
- Added interactive parameter tuning visualizer with three-layer skeleton comparison, HUD, and fingertip highlighting. Supports hot-reload and playback

## [0.2.0] - 2026-04-07

### Added

- Added MP4 video input device with MediaPipe hand tracking
- Added Intel RealSense camera input device with MediaPipe hand tracking
- Added an abstract base class for input devices
- Added a video/RealSense configuration preset
- Added `--video`, `--realsense`, `--show-video` CLI flags to teleop scripts
- Added optional `realsense` and `video` install extras

## [0.1.1] - 2026-02-02

### Changed

- Upgraded to v0.2.1 hand description

### Fixed

- Removed invalid MDX syntax in appendix documentation

## [0.1.0] - 2026-01-21

Initial public release.

### Added

- Added core retargeting system with a unified retargeter interface
- Added adaptive analytical optimizer with Huber loss and hand-written analytical gradients
- Added low-pass filtering for smooth motion output
- Added Apple Vision Pro hand tracking support
- Added MediaPipe recording playback support
- Added input data recording functionality
- Added MuJoCo simulation example
- Added real hardware control example
- Added YAML-based configuration system with per-finger scaling and pinch thresholds

[Unreleased]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.8.3...HEAD
[2026.8.3]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.6.27...v2026.8.3
[2026.6.27]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.6.15...v2026.6.27
[2026.6.15]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.6.10...v2026.6.15
[2026.6.10]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.05.26...v2026.6.10
[2026.05.26]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.05.23...v2026.05.26
[2026.05.23]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.05.18...v2026.05.23
[2026.05.18]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.04.27...v2026.05.18
[2026.04.27]: https://github.com/wuji-technology/wuji-retargeting/compare/v0.2.0...v2026.04.27
[0.2.0]: https://github.com/wuji-technology/wuji-retargeting/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/wuji-technology/wuji-retargeting/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/wuji-technology/wuji-retargeting/releases/tag/v0.1.0

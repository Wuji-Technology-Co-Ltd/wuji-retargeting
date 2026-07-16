# Quest3 Teleop Three-End Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build the first-phase Quest3 hand tracking teleop path from Quest3 app protocol through Orin relay, Control PC input device, existing sim/real teleop entry points, and real-hand safety gating.

**Architecture:** Keep Orin as a raw WebSocket relay and keep retargeting plus safety on the Control PC. Add a package for protocol, Quest26-to-MP21 conversion, relay tools, and safety, then wire `example/teleop_sim.py` and `example/teleop_real.py` to a new `Quest3Device` that preserves the existing `get_fingers_data()` contract.

**Tech Stack:** Python 3.10+, NumPy, PyYAML, websockets, pytest, existing `wuji_retargeting.Retargeter`, existing teleop scripts, Unity C# scaffold docs/source files.

---

### Task 1: Protocol, Codec, and Validation

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/__init__.py`
- Create: `stardust_wuji_quest3_pc_retargeting/protocol/__init__.py`
- Create: `stardust_wuji_quest3_pc_retargeting/protocol/messages.py`
- Create: `stardust_wuji_quest3_pc_retargeting/protocol/validation.py`
- Create: `stardust_wuji_quest3_pc_retargeting/protocol/json_codec.py`
- Test: `tests/test_quest_protocol.py`

- [x] Write tests for valid both-hand `hand_frame`, malformed joint arrays, finite checks, and controller deadman threshold behavior.
- [x] Run `python3 -m pytest tests/test_quest_protocol.py -q` and verify the tests fail because the protocol package does not exist.
- [x] Implement dataclasses, validation, and JSON decode/encode.
- [x] Run `python3 -m pytest tests/test_quest_protocol.py -q` and verify the tests pass.

### Task 2: Quest26 to MP21 Conversion

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/conversion/__init__.py`
- Create: `stardust_wuji_quest3_pc_retargeting/conversion/hand_joint_names.py`
- Create: `stardust_wuji_quest3_pc_retargeting/conversion/quest26_to_mp21.py`
- Create: `configs/quest3/quest26_to_mp21_left.yaml`
- Create: `configs/quest3/quest26_to_mp21_right.yaml`
- Test: `tests/test_quest26_to_mp21.py`

- [x] Write tests for configured mapping, wrist-relative output, independent left/right transforms, invalid-hand zero output, shape validation, and finite validation.
- [x] Run `python3 -m pytest tests/test_quest26_to_mp21.py -q` and verify the tests fail because conversion is missing.
- [x] Implement config loading, axis transform, scale, mapping, and zero fallback for invalid tracking.
- [x] Run `python3 -m pytest tests/test_quest26_to_mp21.py -q` and verify the tests pass.

### Task 3: Quest3Device Input

**Files:**
- Create: `example/input_devices/quest3_device.py`
- Test: `tests/test_quest3_device.py`

- [x] Write tests for `get_fingers_data()` dict shape, invalid-left-only zeroing, invalid-right-only zeroing, stale-frame zeroing both hands, frame age, latest frame, and controller state.
- [x] Run `python3 -m pytest tests/test_quest3_device.py -q` and verify the tests fail because `Quest3Device` is missing.
- [x] Implement a background WebSocket server, message ingestion, latest-frame storage, conversion, stale checks, metadata accessors, and cleanup.
- [x] Run `python3 -m pytest tests/test_quest3_device.py -q` and verify the tests pass.

### Task 4: Safety Filter

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/safety/__init__.py`
- Create: `stardust_wuji_quest3_pc_retargeting/safety/joint_limits.py`
- Create: `stardust_wuji_quest3_pc_retargeting/safety/hand_safety_filter.py`
- Create: `configs/safety/wh110_left.yaml`
- Create: `configs/safety/wh110_right.yaml`
- Test: `tests/test_safety_filter.py`

- [x] Write tests for false deadman rejection, stale-frame hold/disabled behavior, NaN/Inf rejection, out-of-range rejection, and excessive jump limiting or hold.
- [x] Run `python3 -m pytest tests/test_safety_filter.py -q` and verify the tests fail because safety is missing.
- [x] Implement safety state transitions, joint limit loading, qpos validation, max jump behavior, hold, safe-open, and disabled output.
- [x] Run `python3 -m pytest tests/test_safety_filter.py -q` and verify the tests pass.

### Task 5: Teleop Integration

**Files:**
- Modify: `example/teleop_sim.py`
- Modify: `example/teleop_real.py`
- Create: `configs/retargeting/adaptive_analytical_quest3_left.yaml`
- Create: `configs/retargeting/adaptive_analytical_quest3_right.yaml`
- Create: `configs/services/control_pc_default.yaml`
- Test: `tests/test_teleop_quest3_integration.py`

- [x] Write tests that `teleop_sim.py --input quest3` constructs `Quest3Device`, `teleop_real.py --input quest3` defaults to dry-run without `wujihandpy.Hand()`, and `--enable-real-hand` with Quest3 requires a safety config.
- [x] Run `python3 -m pytest tests/test_teleop_quest3_integration.py -q` and verify the tests fail.
- [x] Add Quest3 CLI options and device construction to `teleop_sim.py`.
- [x] Add Quest3 CLI options, dry-run default, hardware opt-in, and safety gating to `teleop_real.py`.
- [x] Run `python3 -m pytest tests/test_teleop_quest3_integration.py -q` and verify the tests pass.

### Task 6: Orin Relay and Mock Sender

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/orin_relay/__init__.py`
- Create: `stardust_wuji_quest3_pc_retargeting/orin_relay/adb_manager.py`
- Create: `stardust_wuji_quest3_pc_retargeting/orin_relay/quest_usb_relay.py`
- Create: `stardust_wuji_quest3_pc_retargeting/orin_relay/relay_status.py`
- Create: `stardust_wuji_quest3_pc_retargeting/tools/__init__.py`
- Create: `stardust_wuji_quest3_pc_retargeting/tools/mock_quest_sender.py`
- Create: `stardust_wuji_quest3_pc_retargeting/tools/run_orin_relay.py`
- Create: `configs/services/orin_relay_default.yaml`

- [x] Implement adb reverse helper, raw bidirectional WebSocket relay with reconnect logging, CLI wrapper, and mock sender emitting valid protocol frames.
- [x] Run targeted protocol tests plus import checks for both CLIs.

### Task 7: Unity Quest3 App Scaffold

**Files:**
- Create: `quest3_app/README.md`
- Create: `quest3_app/UnityProject/Assets/Scripts/HandTrackingProvider.cs`
- Create: `quest3_app/UnityProject/Assets/Scripts/ControllerProvider.cs`
- Create: `quest3_app/UnityProject/Assets/Scripts/TeleopWebSocketClient.cs`
- Create: `quest3_app/UnityProject/Assets/Scripts/FrameSerializer.cs`
- Create: `quest3_app/UnityProject/Assets/Scripts/StatusPanel.cs`

- [x] Add Unity-side scaffold documenting the required app name, WebSocket URL, hello/hand_frame protocol, dual-hand validity, controller grip deadman, and status panel responsibilities.
- [x] Run a syntax-light repository check that the files exist and contain the expected class names.

### Task 8: Full Verification

**Files:**
- All changed files.

- [x] Run `python3 -m pytest tests/test_quest_protocol.py tests/test_quest26_to_mp21.py tests/test_quest3_device.py tests/test_safety_filter.py tests/test_teleop_quest3_integration.py -q`.
- [x] Run import checks for protocol, conversion, safety, relay tools, and `Quest3Device`.
- [x] Run `git status --short` and review the final file set.

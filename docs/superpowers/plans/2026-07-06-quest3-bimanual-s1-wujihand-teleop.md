# Quest3 Bimanual S1 WujiHand Teleop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first dry-run-capable Quest Browser WebXR bimanual teleop path while keeping all real robot outputs opt-in.

**Architecture:** Add a new `stardust_wuji_quest3_pc_retargeting` package with protocol, conversion, safety, arm mapping, gateway, simulation, and runtime modules. Existing retargeting and ROS2/Astribot paths remain optional adapters; the default supervisor path validates WebXR frames, converts hands to MP21, generates bounded dry-run hand and arm outputs, and never touches hardware unless explicit flags are supplied.

**Tech Stack:** Python 3.10+, NumPy, PyYAML, websockets, pytest, static HTML/CSS/JavaScript for Quest Browser.

---

### Task 1: Protocol and JSON Codec

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/protocol/messages.py`
- Create: `stardust_wuji_quest3_pc_retargeting/protocol/validation.py`
- Create: `stardust_wuji_quest3_pc_retargeting/protocol/json_codec.py`
- Test: `tests/test_webxr_protocol.py`

- [ ] Write tests that accept a valid `tracking_frame`, reject bad schema/type, reject NaN/Inf, reject mismatched joint arrays, and round-trip JSON.
- [ ] Run `python3 -m pytest tests/test_webxr_protocol.py -q`; expected result is import failure before implementation.
- [ ] Implement dataclasses, strict validation, finite checks, and JSON encode/decode.
- [ ] Run the same test and confirm it passes.

### Task 2: WebXR to MP21 Conversion

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/conversion/hand_joint_names.py`
- Create: `stardust_wuji_quest3_pc_retargeting/conversion/webxr_to_mp21.py`
- Create: `stardust_wuji_quest3_pc_retargeting/conversion/pose_math.py`
- Create: `configs/quest3_web/webxr_hand_mapping_left.yaml`
- Create: `configs/quest3_web/webxr_hand_mapping_right.yaml`
- Test: `tests/test_webxr_to_mp21.py`

- [ ] Write tests for MP21 shape, wrist-relative output, invalid-hand zero output, missing joint rejection, and YAML-configured scale.
- [ ] Run `python3 -m pytest tests/test_webxr_to_mp21.py -q`; expected result is import failure before implementation.
- [ ] Implement fixed WebXR joint names, default MP21 mapping, config loading, wrist-relative output, and finite validation.
- [ ] Run the same test and confirm it passes.

### Task 3: Safety and State Machine

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/safety/state_machine.py`
- Create: `stardust_wuji_quest3_pc_retargeting/safety/hand_safety_filter.py`
- Create: `stardust_wuji_quest3_pc_retargeting/safety/arm_safety_filter.py`
- Create: `stardust_wuji_quest3_pc_retargeting/safety/teleop_safety.py`
- Create: `configs/safety/quest3_teleop_default.yaml`
- Test: `tests/test_state_machine.py`
- Test: `tests/test_hand_safety_filter.py`
- Test: `tests/test_arm_safety_filter.py`

- [ ] Write tests for IDLE/ARMED/RUNNING/PAUSED/ESTOP/FAULT transitions, hand qpos limits, jump limits, stale hold, arm workspace clipping, and arm max-step limiting.
- [ ] Run the three tests; expected result is import failure before implementation.
- [ ] Implement deterministic state transitions and pure safety filters.
- [ ] Run the three tests and confirm they pass.

### Task 4: Arm Mapper and Supervisor Dry-Run

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/arm_control/arm_mapper.py`
- Create: `stardust_wuji_quest3_pc_retargeting/arm_control/workspace_limits.py`
- Create: `stardust_wuji_quest3_pc_retargeting/runtime/config.py`
- Create: `stardust_wuji_quest3_pc_retargeting/runtime/supervisor.py`
- Create: `stardust_wuji_quest3_pc_retargeting/sim/dryrun_robot.py`
- Test: `tests/test_arm_mapper.py`
- Test: `tests/test_supervisor_dryrun.py`

- [ ] Write tests for clutch-relative arm mapping and a valid frame producing left/right hand qpos plus left/right arm targets in dry-run.
- [ ] Run the tests; expected result is import failure before implementation.
- [ ] Implement relative calibration, bounded dry-run outputs, and config path resolution.
- [ ] Run the tests and confirm they pass.

### Task 5: Gateway, Web Page, and CLI Entrypoints

**Files:**
- Create: `stardust_wuji_quest3_pc_retargeting/web_gateway/static_server.py`
- Create: `stardust_wuji_quest3_pc_retargeting/web_gateway/websocket_relay.py`
- Create: `stardust_wuji_quest3_pc_retargeting/web_gateway/adb_reverse.py`
- Create: `stardust_wuji_quest3_pc_retargeting/sim/mock_webxr_sender.py`
- Create: `stardust_wuji_quest3_pc_retargeting/tools/run_orin_web_gateway.py`
- Create: `stardust_wuji_quest3_pc_retargeting/tools/run_control_pc_supervisor.py`
- Create: `quest3_web/index.html`
- Create: `quest3_web/src/app.js`
- Create: `quest3_web/src/webxr_session.js`
- Create: `quest3_web/src/hand_frame_serializer.js`
- Create: `quest3_web/src/websocket_client.js`
- Create: `quest3_web/src/status_panel.js`
- Create: `quest3_web/src/session_state.js`
- Create: `quest3_web/styles/main.css`
- Create: `example/teleop_quest3_bimanual_sim.py`
- Create: `example/teleop_quest3_bimanual_real.py`

- [ ] Add import tests for all Python CLIs and static file existence checks.
- [ ] Run the import tests; expected result is import failure before implementation.
- [ ] Implement CLI wrappers, mock sender, static WebXR page, and a raw WebSocket relay.
- [ ] Run import tests and targeted unit tests.

### Task 6: Final Verification

**Files:**
- All changed files.

- [ ] Run `python3 -m pytest tests/test_webxr_protocol.py tests/test_webxr_to_mp21.py tests/test_state_machine.py tests/test_hand_safety_filter.py tests/test_arm_safety_filter.py tests/test_arm_mapper.py tests/test_supervisor_dryrun.py -q`.
- [ ] Run `python3 -m compileall stardust_wuji_quest3_pc_retargeting example/teleop_quest3_bimanual_sim.py example/teleop_quest3_bimanual_real.py`.
- [ ] Run `git status --short` and review the final file set without reverting unrelated user changes.

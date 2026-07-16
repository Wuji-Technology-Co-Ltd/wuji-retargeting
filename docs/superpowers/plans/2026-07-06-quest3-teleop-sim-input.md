# Quest3 Teleop Sim Input Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `example/teleop_sim.py --input quest3` work with the current Quest Browser/WebXR protocol.

**Architecture:** Keep `teleop_sim.py` unchanged except for already-existing Quest3 wiring, and make `example/input_devices/quest3_device.py` satisfy that interface. The device owns a small background WebSocket server, ingests `tracking_frame` messages, converts left/right WebXR hands to MP21 arrays, exposes `left_fingers`/`right_fingers`, and reports frame age plus controller deadman status.

**Tech Stack:** Python 3.10+, websockets, NumPy, PyYAML, pytest.

---

### Task 1: Quest3Device teleop_sim contract

**Files:**
- Modify: `example/input_devices/quest3_device.py`
- Test: `tests/test_quest3_device_teleop_sim.py`

- [ ] Write failing tests for constructor compatibility, `left_fingers`/`right_fingers` output, stale zeroing, invalid single-hand zeroing, `from_service_config`, and background WebSocket ingestion.
- [ ] Run `python3 -m pytest tests/test_quest3_device_teleop_sim.py -q`; expect failures against the current minimal device.
- [ ] Implement the compatible Quest3Device with background WebSocket server, lock-protected latest frame storage, config loading, frame age, controller state, and cleanup.
- [ ] Run `python3 -m pytest tests/test_quest3_device_teleop_sim.py -q`; expect pass.

### Task 2: Regression verification

**Files:**
- All changed files.

- [ ] Run `python3 -m pytest -q`.
- [ ] Run `python3 example/teleop_quest3_bimanual_sim.py`.

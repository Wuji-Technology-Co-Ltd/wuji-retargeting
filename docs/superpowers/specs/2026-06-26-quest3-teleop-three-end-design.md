# Quest3 Three-End Teleop Integration Design

## Goal

Build the Quest3 hand tracking teleoperation system entirely inside the
`wuji-retargeting` repository:

```text
Quest3 Unity App -> Orin relay -> Control PC quest3 input device
  -> existing teleop_sim.py / teleop_real.py -> wuji-retargeting -> Wuji Hand
```

The first implementation phase replaces the Wuji Glove input source with Quest3
optical hand tracking. It does not replace the Stardust S1 official web control
path, and it does not make Orin responsible for retargeting or hand safety.

## Repository Boundary

All code, configs, tests, and docs for the three endpoints live under:

```text
/home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting
```

No implementation files are placed in sibling folders or external projects.

Target layout:

```text
wuji-retargeting/
  quest3_app/
    README.md
    UnityProject/
      Assets/
        Scripts/
          HandTrackingProvider.cs
          ControllerProvider.cs
          TeleopWebSocketClient.cs
          FrameSerializer.cs
          StatusPanel.cs

  stardust_wuji_quest3_pc_retargeting/
    protocol/
      messages.py
      validation.py
      json_codec.py
    conversion/
      hand_joint_names.py
      quest26_to_mp21.py
    orin_relay/
      adb_manager.py
      quest_usb_relay.py
      relay_status.py
    safety/
      hand_safety_filter.py
      joint_limits.py
    tools/
      mock_quest_sender.py
      run_orin_relay.py

  example/
    input_devices/
      quest3_device.py
    teleop_sim.py
    teleop_real.py

  configs/
    quest3/
      quest26_to_mp21_left.yaml
      quest26_to_mp21_right.yaml
    retargeting/
      adaptive_analytical_quest3_left.yaml
      adaptive_analytical_quest3_right.yaml
    safety/
      wh110_left.yaml
      wh110_right.yaml
    services/
      orin_relay_default.yaml
      control_pc_default.yaml

  tests/
    test_quest_protocol.py
    test_quest26_to_mp21.py
    test_quest3_device.py
    test_teleop_quest3_integration.py
    test_safety_filter.py
```

## Chosen Architecture

The implementation uses the existing teleop entry points instead of creating a
separate Control PC live runner.

```text
Quest3 Unity App
  sends Quest26 + controller/grip over ws://127.0.0.1:9001
    |
    | USB-C + adb reverse
    v
Orin relay
  forwards raw WebSocket messages only
    |
    | Ethernet
    v
Control PC Quest3Device
  validates protocol
  stores latest frame
  converts left Quest26 -> left MP21
  converts right Quest26 -> right MP21
    |
    v
example/teleop_sim.py or example/teleop_real.py
  selects one hand with --hand left|right
  calls existing Retargeter
    |
    v
MuJoCo sim or real Wuji Hand
```

This preserves the existing `InputDevice.get_fingers_data()` contract:

```python
{
    "left_fingers": np.ndarray,   # shape (21, 3)
    "right_fingers": np.ndarray,  # shape (21, 3)
}
```

## Endpoint Responsibilities

### Quest3 App

The Quest3 app is a minimal Unity/Android app named `Quest3HandStreamer`.

Responsibilities:

- Read both left and right Quest3 hand tracking joints every frame.
- Send both hands in every `hand_frame`.
- Report `left_valid` and `right_valid` independently.
- Read controller grip values.
- Compute global deadman:

```text
deadman = max(left_grip, right_grip) >= grip_deadman_threshold
```

The default threshold is:

```text
grip_deadman_threshold = 0.5
```

- Connect to:

```text
ws://127.0.0.1:9001
```

- Send `hello` once after connection.
- Send `hand_frame` continuously at tracking rate.
- Show a simple status panel:
  - WebSocket connected/disconnected
  - left/right tracking valid
  - send FPS
  - deadman state

The Quest3 app does not:

- Convert Quest26 to MP21.
- Run retargeting.
- Produce qpos.
- Control Wuji Hand directly.
- Implement a complex dashboard.

### Orin Relay

Orin is only a USB-to-Ethernet relay node.

Responsibilities:

- Ensure or instruct:

```bash
adb reverse tcp:9001 tcp:9001
```

- Listen on:

```text
127.0.0.1:9001
```

- Connect to:

```text
ws://<CONTROL_PC_IP>:9001
```

- Forward WebSocket messages in both directions.
- Reconnect after Quest3 or Control PC disconnects.
- Log connection state, forwarded frame count, and last error.

Orin does not:

- Parse Quest26 joints for control decisions.
- Convert to MP21.
- Run `wuji-retargeting`.
- Apply safety limits.
- Connect to Wuji Hand.

### Control PC Quest3Device

`example/input_devices/quest3_device.py` is the main Control PC integration
point.

Responsibilities:

- Listen for WebSocket messages from Orin.
- Decode and validate protocol messages.
- Store the latest `QuestHandFrame`.
- Convert left and right hands independently:

```text
left_joints  -> left_converter  -> left_fingers
right_joints -> right_converter -> right_fingers
```

- Implement the existing teleop input-device method:

```python
def get_fingers_data(self) -> dict:
    return {
        "left_fingers": left_mp21,
        "right_fingers": right_mp21,
    }
```

- Expose extra safety metadata for `teleop_real.py`:

```python
get_controller_state()
get_frame_age_sec()
get_latest_frame()
```

Dual-hand behavior:

- Quest3 always sends both hands.
- Left and right valid flags are independent.
- Invalid left hand returns zero MP21 only for left.
- Invalid right hand returns zero MP21 only for right.
- A stale whole frame makes both hands zero.
- `--hand left` consumes only `left_fingers`.
- `--hand right` consumes only `right_fingers`.

## Protocol

Every message contains:

```json
{
  "version": 1,
  "type": "..."
}
```

`hand_frame`:

```json
{
  "version": 1,
  "type": "hand_frame",
  "seq": 1,
  "timestamp_ns": 123456789,
  "left_valid": true,
  "right_valid": true,
  "left_joints": [[0.0, 0.0, 0.0]],
  "right_joints": [[0.0, 0.0, 0.0]],
  "controller": {
    "deadman": true,
    "left_grip": 0.0,
    "right_grip": 0.0,
    "left_trigger": 0.0,
    "right_trigger": 0.0,
    "button_a": false,
    "button_b": false,
    "button_x": false,
    "button_y": false
  }
}
```

Requirements:

- `left_joints` is always 26x3.
- `right_joints` is always 26x3.
- Units are meters.
- Quest3 sends both hands every frame.
- Invalid tracking is expressed with `left_valid` or `right_valid`, not by
  omitting fields.
- Control PC records its own receive time for stale-frame checks.

## Quest26 to MP21 Conversion

Conversion is config-driven and uses separate left/right YAML files:

```text
configs/quest3/quest26_to_mp21_left.yaml
configs/quest3/quest26_to_mp21_right.yaml
```

Algorithm per hand:

1. If valid flag is false, return zero MP21.
2. Validate shape is 26x3.
3. Validate all values are finite.
4. Map configured Quest26 names to MP21 names.
5. Apply configured 3x3 axis transform.
6. Apply configured scale.
7. Make points wrist-relative.
8. Return `float32` array with shape 21x3.

Left and right configs remain separate because the Quest coordinate frame or
handedness correction may differ by side.

## Teleop Integration

### `example/teleop_sim.py`

Add input type:

```text
--input quest3
```

Add Quest3 options:

```text
--quest-host 0.0.0.0
--quest-port 9001
--quest-left-config ../configs/quest3/quest26_to_mp21_left.yaml
--quest-right-config ../configs/quest3/quest26_to_mp21_right.yaml
--grip-deadman-threshold 0.5
--require-deadman
```

Default sim behavior:

- `--hand left` reads `left_fingers`.
- `--hand right` reads `right_fingers`.
- Missing, stale, or invalid selected-hand data skips retargeting for that frame.
- Deadman is visible/logged but not required unless `--require-deadman` is set.

Example:

```bash
mjpython example/teleop_sim.py \
  --input quest3 \
  --hand right \
  --config ../configs/retargeting/adaptive_analytical_quest3_right.yaml \
  --quest-host 0.0.0.0 \
  --quest-port 9001
```

### `example/teleop_real.py`

Add input type:

```text
--input quest3
```

Add Quest3 and safety options:

```text
--quest-host 0.0.0.0
--quest-port 9001
--quest-left-config ../configs/quest3/quest26_to_mp21_left.yaml
--quest-right-config ../configs/quest3/quest26_to_mp21_right.yaml
--safety-config ../configs/safety/wh110_right.yaml
--grip-deadman-threshold 0.5
--enable-real-hand
--safe-open-on-deadman-release
```

Default real behavior:

- Without `--enable-real-hand`, run dry-run mode.
- Dry-run mode does not initialize `wujihandpy.Hand()`.
- Dry-run mode prints selected hand, qpos preview, safety state, deadman, and FPS.
- With `--enable-real-hand`, initialize hardware and send commands only when
  safety state is `ACTIVE`.

The real path must not send qpos when:

- No Quest3 frame has arrived.
- Latest frame is stale.
- Selected hand tracking is invalid.
- Grip deadman is false.
- Retargeting output contains NaN or Inf.
- qpos is outside configured joint limits.
- qpos jump is too large.
- Hardware controller is not initialized.

On `Ctrl-C` or process exit:

```python
hand.write_joint_enabled(False)
```

must run when real hardware was enabled.

## Safety Design

Safety runs on the Control PC because that is where Wuji Hand is connected.

State names:

```text
DISABLED
ARMED
ACTIVE
HOLD
SAFE_OPEN
ERROR
```

First-phase behavior:

- `frame_age < 200 ms`: allow current safe qpos if all other checks pass.
- `200 ms <= frame_age < 1000 ms`: hold last safe qpos or safe-open depending
  on `--safe-open-on-deadman-release`.
- `frame_age >= 1000 ms`: enter `DISABLED`.
- `deadman == false`: enter `HOLD` by default.
- NaN/Inf retarget output: reject frame and enter `ERROR`.
- qpos out of range: reject frame and enter `HOLD`.
- qpos jump too large: limit delta or enter `HOLD`.

The grip deadman is global in phase one:

```text
deadman = left_grip >= 0.5 OR right_grip >= 0.5
```

Per-hand deadman can be added later if operational testing shows a need:

```text
left_deadman = left_grip >= 0.5
right_deadman = right_grip >= 0.5
```

## Validation Plan

Validation proceeds in stages.

### Stage 1: Local Mock to Sim

Run sim on Control PC:

```bash
mjpython example/teleop_sim.py \
  --input quest3 \
  --hand right \
  --config ../configs/retargeting/adaptive_analytical_quest3_right.yaml \
  --quest-host 0.0.0.0 \
  --quest-port 9001
```

Send mock frames:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.mock_quest_sender \
  --url ws://127.0.0.1:9001
```

Success criteria:

- `quest3_device` receives frames.
- Quest26 to MP21 conversion works.
- `teleop_sim.py` drives the selected hand.
- `--hand left` and `--hand right` select different hand data.

### Stage 2: Orin Relay to Sim

On Orin:

```bash
adb reverse tcp:9001 tcp:9001

python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_orin_relay \
  --listen-host 127.0.0.1 \
  --listen-port 9001 \
  --control-pc-url ws://<CONTROL_PC_IP>:9001
```

Control PC keeps running `teleop_sim.py --input quest3`.

Success criteria:

- Relay forwards frames to Control PC.
- Reconnect works after disconnect.
- Orin does not need `wuji-retargeting` or hardware access.

### Stage 3: Quest3 App to Sim

Quest3 app connects to:

```text
ws://127.0.0.1:9001
```

Success criteria:

- Quest3 sends both hands every frame.
- Left and right valid flags match visible tracking.
- Grip deadman changes at threshold 0.5.
- Left and right MP21 data do not swap.
- Sim follows the selected hand.

### Stage 4: Real Dry-Run

Run without real hardware opt-in:

```bash
python3 example/teleop_real.py \
  --input quest3 \
  --hand right \
  --config ../configs/retargeting/adaptive_analytical_quest3_right.yaml \
  --safety-config ../configs/safety/wh110_right.yaml
```

Success criteria:

- `wujihandpy.Hand()` is not initialized.
- qpos preview is printed.
- Safety state changes correctly with grip, stale frames, and invalid hand data.

### Stage 5: Real Hardware

Run with explicit opt-in:

```bash
python3 example/teleop_real.py \
  --input quest3 \
  --hand right \
  --config ../configs/retargeting/adaptive_analytical_quest3_right.yaml \
  --safety-config ../configs/safety/wh110_right.yaml \
  --enable-real-hand
```

Success criteria:

- Hardware only moves when selected hand is valid, frame is fresh, and grip
  deadman is active.
- Releasing grip stops new motion.
- Disconnecting Quest3 stops new motion.
- Process exit disables joints.

## Test Coverage

Unit tests:

- Protocol validation accepts both-hand `hand_frame`.
- Protocol validation rejects malformed joint arrays.
- Controller parsing computes grip deadman at threshold 0.5.
- Quest26 to MP21 conversion returns independent left/right arrays.
- Invalid left hand zeroes only left output.
- Invalid right hand zeroes only right output.
- Stale frame zeroes both outputs.
- `Quest3Device.get_fingers_data()` returns the existing teleop dict shape.
- Safety filter rejects false deadman.
- Safety filter rejects stale frame.
- Safety filter rejects NaN/Inf qpos.
- Safety filter rejects or limits excessive qpos jumps.

Integration tests:

- `teleop_sim.py --input quest3` constructs a Quest3 device.
- `teleop_real.py --input quest3` defaults to dry-run without initializing
  hardware.
- `teleop_real.py --input quest3 --enable-real-hand` requires safety config.

## Non-Goals

First phase does not implement:

- Stardust S1 web control replacement.
- Reverse engineering of the Stardust web protocol.
- Orin-side retargeting.
- Orin-side Wuji Hand control.
- Haptic feedback.
- Learning-based retargeting.
- Unified dual-hand real-hardware runner.
- Complex dashboard UI.

Dual-hand real hardware can be implemented later as either two single-hand
processes or a dedicated `teleop_dual_real.py` after the single-hand path is
validated.
